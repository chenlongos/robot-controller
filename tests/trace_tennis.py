"""网球自动追踪测试程序"""
import sys
import os
import signal
import json
import asyncio
import threading
import fractions

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import time
import logging
from src.controller.vision_module import VisionModule
from src.controller.base_controller import BaseController
from src.abstract.base_factory import BaseFactory
import src.base
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera
from _robot_select import select_robot

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
)
from aiortc.contrib.media import MediaRelay
from av import VideoFrame

running = True
pcs = set()
relay = None
video_track = None

class VideoTransformTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, track):
        super().__init__()
        self.track = track

    async def recv(self):
        frame = await self.track.recv()
        return frame


class OpenCVVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.frame_queue = asyncio.Queue(maxsize=5)
        self._start_time = time.time()
        self._frame_count = 0
        self._fps = 30

    async def recv(self):
        frame = await self.frame_queue.get()
        return frame

    def push_frame(self, cv_frame):
        self._frame_count += 1
        
        frame = VideoFrame.from_ndarray(cv_frame, format="bgr24")
        frame.pts = int((time.time() - self._start_time) * 1e6)
        frame.time_base = fractions.Fraction(1, 1000000)
        
        try:
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self.frame_queue.put_nowait(frame)
        except asyncio.QueueFull:
            pass


def signal_handler(signum, frame):
    global running
    running = False
    logging.info(f"收到信号 {signum}，正在退出...")

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def load_calibration_params(robot_type: str, config_dir: str = 'config') -> dict:
    """从校准文件加载距离计算参数
    
    Args:
        robot_type: 机器人类型（如 'aka01b'）
        config_dir: 校准文件目录
        
    Returns:
        dict: 包含 M 和 C 的字典，若文件不存在则返回默认值
    """
    import os
    import yaml
    
    calibration_file = os.path.join(config_dir, f'calibration_{robot_type}_distance.yaml')
    
    if os.path.exists(calibration_file):
        try:
            with open(calibration_file, 'r') as f:
                data = yaml.safe_load(f)
                return {
                    'M': data.get('M', 6042.48),
                    'C': data.get('C', -2.83)
                }
        except Exception as e:
            print(f"加载校准文件失败: {e}")
    
    return {'M': 6042.48, 'C': -2.83}


def calculate_distance(bbox_width: int, M: float = None, C: float = None) -> float:
    """根据检测框宽度估算真实世界距离（米）
    
    使用公式: D(cm) = M / P + C(cm)
    
    Args:
        bbox_width: 检测框宽度（像素）
        M: 校准参数M（可选，若为None则使用默认值）
        C: 校准参数C（可选，若为None则使用默认值）
        
    Returns:
        estimated_distance: 估算的距离（米）
    """
    if bbox_width <= 0:
        return float('inf')
    
    if M is None:
        M = 6042.48
    if C is None:
        C = -2.83
    
    distance_cm = M / bbox_width + C
    distance_m = distance_cm / 100.0
    
    return distance_m


def calculate_observation(frame_width: int, frame_height: int, detections: list, config, 
                          M: float = None, C: float = None) -> dict:
    """计算追踪所需的观测数据
    
    Args:
        frame_width: 图像宽度
        frame_height: 图像高度
        detections: 检测结果列表
        config: 配置对象
        M: 距离校准参数M（可选）
        C: 距离校准参数C（可选）
        
    Returns:
        observation: 包含 target_offset_x, target_offset_y, target_distance 的观测数据
    """
    if not detections:
        return {"target_offset_x": 0.0, "target_offset_y": 0.0, "target_distance": float('inf')}
    
    best_detection = max(detections, key=lambda d: d.get('score', 0))
    
    bbox_center_x = best_detection['x'] + best_detection['w'] / 2
    bbox_center_y = best_detection['y'] + best_detection['h'] / 2
    
    image_center_x = frame_width / 2
    image_center_y = frame_height / 2
    
    target_offset_x = bbox_center_x - image_center_x
    target_offset_y = bbox_center_y - image_center_y
    
    bbox_width = best_detection['w']
    bbox_height = best_detection['h']
    
    target_distance = calculate_distance(max(bbox_width, bbox_height), M, C)
    
    return {
        "target_offset_x": target_offset_x,
        "target_offset_y": target_offset_y,
        "target_distance": target_distance,
        "bbox_width": bbox_width,
        "bbox_height": bbox_height,
        "score": best_detection.get('score', 0)
    }


def get_status(observation: dict, controller_config: dict) -> str:
    """根据观测数据确定下一状态
    
    状态转换逻辑：
    - 当推理结果为 null 时进入 search 模式
    - 当球的距离较远时进入 track 模式（距离 > approach_threshold）
    - 当球的距离较近时进入 approach 模式（距离 <= approach_threshold）
    - 当球的范围满足 (target_x - threshold_x, target_x + threshold_x) 且
      (target_distance - threshold_d, target_distance + threshold_d) 时进入 hold 模式
    
    Args:
        observation: 观测数据
        controller_config: 控制器配置参数
        
    Returns:
        next_state: 下一状态
    """
    STATE_SEARCH = 'search'
    STATE_TRACK = 'track'
    STATE_APPROACH = 'approach'
    STATE_HOLD = 'hold'
    
    distance = observation.get('target_distance', float('inf'))
    error_x = observation.get('target_offset_x', 0.0)
    has_target = distance != float('inf')
    
    if not has_target:
        return STATE_SEARCH
    
    approach_threshold = controller_config.get('approach_threshold', 1.5)
    target_x = controller_config.get('target_x', 0.0)
    threshold_x = controller_config.get('threshold_x', 10.0)
    target_distance = controller_config.get('target_distance', 0.2)
    threshold_d = controller_config.get('threshold_d', 0.01)
    
    angle_done = (error_x >= target_x - threshold_x) and (error_x <= target_x + threshold_x)
    distance_done = (distance >= target_distance - threshold_d) and (distance <= target_distance + threshold_d)
    
    if angle_done and distance_done:
        return STATE_HOLD
    
    if distance > approach_threshold:
        return STATE_TRACK
    
    return STATE_APPROACH


def draw_tracking_info(frame: cv2.Mat, detections: list, observation: dict, config) -> cv2.Mat:
    """在图像上绘制追踪信息"""
    frame_copy = frame.copy()
    height, width = frame_copy.shape[:2]
    
    if detections:
        best_detection = max(detections, key=lambda d: d.get('score', 0))
        x, y, w, h = best_detection['x'], best_detection['y'], best_detection['w'], best_detection['h']
        
        cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        distance = observation.get("target_distance", float('inf'))
        cv2.putText(frame_copy, f"Distance: {distance:.1f}m", (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        cv2.circle(frame_copy, (center_x, center_y), 5, (0, 255, 0), -1)
        
    offset_x = observation.get("target_offset_x", 0)
    distance = observation.get("target_distance", float('inf'))
    
    status_text = f"Offset X: {offset_x:.1f} | Distance: {distance:.1f}m"
    cv2.putText(frame_copy, status_text, (10, height - 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    mode_text = "TRACKING" if detections else "SEARCHING"
    mode_color = (0, 255, 0) if detections else (0, 165, 255)
    cv2.putText(frame_copy, mode_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
    
    return frame_copy


ROOT = os.path.dirname(__file__)


async def index(request: web.Request) -> web.Response:
    content = open(os.path.join(ROOT, "tennis_client.html"), "r").read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request: web.Request) -> web.Response:
    content = open(os.path.join(ROOT, "tennis_client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)


def force_codec(pc: RTCPeerConnection, sender: RTCRtpSender, forced_codec: str) -> None:
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )


async def offer(request: web.Request) -> web.Response:
    global relay, video_track
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        logger.info("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    if video_track is None:
        video_track = OpenCVVideoTrack()

    if relay is None:
        relay = MediaRelay()

    video = relay.subscribe(video_track)

    if video:
        video_sender = pc.addTrack(video)
        force_codec(pc, video_sender, "video/H264")

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )


async def on_shutdown(app: web.Application) -> None:
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def start_webrtc_server(port: int = 8080):
    async def run_server():
        app = web.Application()
        app.on_shutdown.append(on_shutdown)
        app.router.add_get("/", index)
        app.router.add_get("/tennis_client.js", javascript)
        app.router.add_post("/offer", offer)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", port)
        await site.start()
        logger.info(f"WebRTC server started on port {port}")

        while running:
            await asyncio.sleep(1)

        await runner.cleanup()

    loop = asyncio.new_event_loop()
    threading.Thread(target=loop.run_forever, daemon=True).start()
    asyncio.run_coroutine_threadsafe(run_server(), loop)


def main():
    """主程序入口"""
    global running, video_track
    robot_name = select_robot()
    if robot_name is None:
        return
    config = load_config(robot_name=robot_name)
    
    camera = None
    vision = None
    base = None
    controller = None
    
    try:
        calibration_params = load_calibration_params(robot_name)
        logger.info(f"加载校准参数: M={calibration_params['M']:.4f}, C={calibration_params['C']:.4f}")
        
        camera = USBCamera({
            'device_id': config.device.hardware.camera.device_id,
            'resolution': config.device.hardware.camera.resolution,
            'frame_rate': config.device.hardware.camera.frame_rate
        })
        
        if not camera.is_opened():
            logger.error("摄像头未打开，退出程序")
            return
        
        logger.info("正在加载视觉模型...")
        vision = VisionModule(config)
        
        logger.info("正在初始化真实底盘...")
        base_config = config.device.hardware.base
        base_type = base_config.type
        base = BaseFactory.create_base(base_type, base_config)
        logger.info(f"底盘类型: {base_type}, 驱动: {base_config.driver}")
        logger.info(f"轮径: {base_config.wheel_radius}m, 轮距: {base_config.wheel_base}m")
        logger.info(f"最大线速度: {base_config.max_linear_speed}m/s, 最大角速度: {base_config.max_angular_speed}rad/s")
        
        controller_config = {
            'search_rotation_speed': config.control.search_rotation_speed,
            'kp_angle': 0.005,
            'max_linear_speed': base_config.max_linear_speed,
            'approach_threshold': 0.9,
            'max_speed': base_config.max_linear_speed * 0.75,
            'target_x': 0.0,
            'threshold_x': 10.0,
            'target_distance': 0.2,
            'threshold_d': 0.01,
            'approach_kp': 0.6,
            'approach_ki': 0.01,
            'approach_kd': 0.01,
            'approach_kp_angle': 0.002,
        }
        
        controller = BaseController(base, controller_config)
        
        logger.info("启动 WebRTC 服务器...")
        start_webrtc_server(port=8080)
        
        logger.info("网球追踪系统已启动")
        logger.info("WebRTC 推流地址: http://<机器人IP>:8080")
        logger.info("按 Ctrl+C 退出")
        
        frame_count = 0
        total_detections = 0
        start_time = time.time()
        
        STATE_SEARCH = 'search'
        STATE_TRACK = 'track'
        STATE_APPROACH = 'approach'
        STATE_HOLD = 'hold'
        
        current_state = STATE_SEARCH
        search_start_time = time.time()
        
        stream_frame_interval = 1.0 / 15.0
        last_stream_time = time.time()
        
        while running:
            frame_start_time = time.time()
            
            frame = camera.capture(flush_frames=1)
            if frame is None:
                logger.warning("帧捕获失败")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            
            results = vision.infer(frame)
            
            if results:
                total_detections += len(results)
            
            height, width = frame.shape[:2]
            observation = calculate_observation(width, height, results, config,
                                               M=calibration_params['M'],
                                               C=calibration_params['C'])
            
            next_state = get_status(observation, controller_config)
            
            if next_state != current_state:
                if next_state == STATE_APPROACH:
                    controller.reset_pid()
                if next_state == STATE_SEARCH:
                    search_start_time = time.time()
                logger.info(f"状态转换: {current_state} -> {next_state}")
                current_state = next_state
            
            if current_state == STATE_SEARCH:
                command = controller.search()
                w = command.get('w', 0.0)
                if time.time() - search_start_time > 10:
                    search_start_time = time.time()
                    logger.info(f"搜索中: speed_w={w:.3f}")
            
            elif current_state == STATE_TRACK:
                command = controller.track(observation)
                x = command.get('x', 0.0)
                w = command.get('w', 0.0)
                logger.info(f"追踪中: score={observation['score']:.2f}, "
                           f"offset_x={observation['target_offset_x']:.1f}, "
                           f"distance={observation['target_distance']:.1f}m, "
                           f"speed_x={x:.3f}, speed_w={w:.3f}")
            
            elif current_state == STATE_APPROACH:
                command = controller.track(observation)
                x = command.get('x', 0.0)
                w = command.get('w', 0.0)
                logger.info(f"接近中: offset_x={observation['target_offset_x']:.1f}, "
                           f"distance={observation['target_distance']:.1f}m, "
                           f"speed_x={x:.3f}, speed_w={w:.3f}")
            
            elif current_state == STATE_HOLD:
                controller.stop()
                logger.info(f"保持中")
            
            current_time = time.time()
            if current_time - last_stream_time >= stream_frame_interval:
                display_frame = draw_tracking_info(frame, results, observation, config)
                display_frame = cv2.putText(display_frame, f"State: {current_state.upper()}", 
                                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                if video_track:
                    video_track.push_frame(display_frame)
                
                last_stream_time = current_time
            
            elapsed = time.time() - frame_start_time
            if elapsed < 0.033:
                time.sleep(0.033 - elapsed)
        
        elapsed_time = time.time() - start_time
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        logger.info(f"\n追踪结束")
        logger.info(f"总帧数: {frame_count}")
        logger.info(f"总检测目标数: {total_detections}")
        logger.info(f"平均FPS: {fps:.1f}")
        
    except Exception as e:
        logger.error(f"程序异常: {e}", exc_info=True)
    
    finally:
        logger.info("清理资源...")
        
        if controller:
            controller.stop()
        
        if vision:
            vision.release()
        
        if camera:
            camera.release()
        
        if base:
            base.cleanup()


if __name__ == "__main__":
    main()