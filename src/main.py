# src/main.py

import os
import logging
import time
import cv2
import yaml
import json
import asyncio
import threading
import fractions
from dataclasses import dataclass, field
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera
from src.controller.vision_module import VisionModule
from src.controller.base_controller import BaseController
from src.controller.arm_controller import ArmController
from src.abstract.base_factory import BaseFactory
from src.abstract.camera_factory import CameraFactory
from src.abstract.arm_factory import ArmFactory
from src.state_machine import StateMachine, RobotStatus
import src.base

try:
    from aiohttp import web
    from aiortc import (
        MediaStreamTrack,
        RTCPeerConnection,
        RTCRtpSender,
        RTCSessionDescription,
    )
    from aiortc.contrib.media import MediaRelay
    from av import VideoFrame
    HAS_WEBRTC = True
except ImportError:
    HAS_WEBRTC = False
    web = None
    MediaStreamTrack = object
    RTCPeerConnection = object
    RTCRtpSender = object
    RTCSessionDescription = object
    MediaRelay = None
    VideoFrame = None

pcs = set()
relay = None
video_track = None

# 加载配置，默认机器人是 aka00v4-rk3576
config = load_config(robot_name="aka00v4-rk3576")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

if HAS_WEBRTC:
    class OpenCVVideoTrack(MediaStreamTrack):
        kind = "video"

        def __init__(self):
            super().__init__()
            self.frame_queue = asyncio.Queue(maxsize=5)
            self._start_time = time.time()
            self._frame_count = 0

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
            logging.info("Connection state is %s" % pc.connectionState)
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


    async def index(request: web.Request) -> web.Response:
        html_content = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC Robot Tracking</title>
    <style>
    button {
        padding: 8px 16px;
    }

    video {
        width: 100%;
        max-width: 640px;
    }

    .option {
        margin-bottom: 8px;
    }

    #media {
        max-width: 640px;
    }
    </style>
</head>
<body>

<div class="option">
    <input id="use-stun" type="checkbox"/>
    <label for="use-stun">Use STUN server</label>
</div>
<button id="start" onclick="start()">Start</button>
<button id="stop" style="display: none" onclick="stop()">Stop</button>

<div id="media">
    <h2>Robot Tracking Stream</h2>
    <video id="video" autoplay="true" playsinline="true"></video>
</div>

<script>
var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        return response.json();
    }).then((answer) => {
        return pc.setRemoteDescription(answer);
    }).catch((e) => {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (document.getElementById('use-stun').checked) {
        config.iceServers = [{ urls: ['stun:stun.l.google.com:19302'] }];
    }

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';

    setTimeout(() => {
        pc.close();
    }, 500);
}
</script>
</body>
</html>"""
        return web.Response(content_type="text/html", text=html_content)


    def start_webrtc_server(port: int = 8080):
        async def run_server():
            app = web.Application()
            app.on_shutdown.append(on_shutdown)
            app.router.add_get("/", index)
            app.router.add_post("/offer", offer)

            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, "0.0.0.0", port)
            await site.start()
            logging.info(f"WebRTC server started on port {port}")

            while True:
                await asyncio.sleep(1)

            await runner.cleanup()

        loop = asyncio.new_event_loop()
        threading.Thread(target=loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(run_server(), loop)


    def draw_tracking_info(frame: cv2.Mat, detections: list, bucket_detections: list, 
                           state: str, observation: dict) -> cv2.Mat:
        """在图像上绘制追踪信息"""
        frame_copy = frame.copy()
        
        if detections:
            best_detection = max(detections, key=lambda d: d.get('score', 0))
            x, y, w, h = best_detection['x'], best_detection['y'], best_detection['w'], best_detection['h']
            
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            distance = observation.get("tennis_distance", float('inf'))
            cv2.putText(frame_copy, f"Distance: {distance:.1f}m", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            cv2.circle(frame_copy, (center_x, center_y), 5, (0, 255, 0), -1)
        
        if bucket_detections:
            best_detection = max(bucket_detections, key=lambda d: d.get('w', 0))
            x, y, w, h = best_detection['x'], best_detection['y'], best_detection['w'], best_detection['h']
            
            cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame_copy, f"Bucket", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        state_text = str(state).split(".")[-1] if hasattr(state, "__class__") else str(state)
        cv2.putText(frame_copy, f"State: {state_text}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return frame_copy

# 根据配置设置日志
logging.basicConfig(
    level=getattr(logging, config.system.log_level),
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('th.log', mode='w'),
        logging.StreamHandler()
    ]
)

# 注册工厂类
CameraFactory.register_camera("usb", USBCamera)


def load_calibration_params(robot_type: str, config_dir: str = 'config') -> dict:
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
            logging.error(f"加载校准文件失败: {e}")
    
    return {'M': 6042.48, 'C': -2.83}


def calculate_distance(bbox_width: int, M: float = None, C: float = None) -> float:
    if bbox_width <= 0:
        return float('inf')
    
    if M is None:
        M = 6042.48
    if C is None:
        C = -2.83
    
    distance_cm = M / bbox_width + C
    distance_m = distance_cm / 100.0
    
    return distance_m


@dataclass
class Robot:
    """机器人类 - 通过配置文件创建，无需硬编码"""
    status: str = "chase_tennis"  # 机器人状态
    box_cur_width: int = 0
    box_cur_height: int = 0
    box_cur_x: int = 0
    frame_height: int = 0
    target_type: str = "tennis"  # 当前跟踪目标类型: tennis, bucket

    controller: BaseController = field(init=False)  # 使用控制器类型
    arm_controller: ArmController = field(init=False)  # 机械臂控制器
    camera = None

    def __post_init__(self):
        """根据配置初始化机器人底盘和机械臂"""
        try:
            base_config = config.device.hardware.base
            base_type = base_config.type
            base = BaseFactory.create_base(base_type, base_config)
            self.controller = BaseController(base, config.control)
            logging.info("底盘控制器初始化成功")
        except Exception as e:
            logging.error(f"底盘控制器初始化失败: {e}")
            raise
        
        try:
            arm_config = config.device.hardware.arm
            arm_type = arm_config.type
            arm = ArmFactory.create_arm(arm_type, arm_config)
            arm.connect()
            self.arm_controller = ArmController(arm, robot_name=config.system.robot_id)
            logging.info("机械臂控制器初始化成功")
        except Exception as e:
            logging.error(f"机械臂控制器初始化失败: {e}")
            raise
    
    def set_motor_speed(self, result):
        """根据视觉检测结果设置电机速度"""
        IMG_WIDTH = config.device.parameters.frame_width
        MAX_SPEED = config.control.max_speed
        MIN_SPEED = MAX_SPEED // config.control.min_speed_ratio
        WHEEL_BASE = config.control.wheel_base
        TARGET_X = IMG_WIDTH // 2
        
        if self.target_type == "tennis":
            TARGET_W = int(config.vision.tennis_width_far * 0.6 + config.vision.tennis_width_near * 0.4)
        else:
            TARGET_W = 100

        Kp_dist = config.control.kp_dist
        Kp_angle = config.control.kp_angle

        result_sorted = sorted(result, key=lambda x: x['w'], reverse=True)
        box = result_sorted[0]
        x, w, h = box["x"], box["w"], box["h"]
        self.box_cur_height = h
        self.box_cur_x = x
        self.box_cur_width = w
        logging.debug("(box_cur_x, box_cur_width, box_cur_height) ==> %d, %d, %d", 
                     self.box_cur_x, self.box_cur_width, self.box_cur_height)

        if self.target_type == "tennis" and config.vision.tennis_width_far < self.box_cur_width < config.vision.tennis_width_near:
            self.controller.stop()
            return 0, 0

        error_x = (x + w / 2) - TARGET_X
        error_w = w - TARGET_W

        raw_v = -Kp_dist * error_w
        raw_omega = -Kp_angle * error_x

        turn_factor = abs(error_x) / (IMG_WIDTH / 2)
        if turn_factor > 0.8:
            max_v = MIN_SPEED * 0.3
        else:
            max_v = MAX_SPEED

        v = max(min(raw_v, max_v), -max_v)

        if abs(v) < MIN_SPEED and abs(v) > 0:
            v = MIN_SPEED if v > 0 else -MIN_SPEED

        diff_speed = raw_omega * WHEEL_BASE

        self.controller.move(v, 0.0, diff_speed)

        return int(v), int(diff_speed)
    
    def idle(self):
        """空闲状态：旋转搜索"""
        self.controller.search()
    
    def execute_arm_action(self, action_name: str):
        """执行机械臂动作序列"""
        logging.info(f"执行机械臂动作: {action_name}")
        result = self.arm_controller.execute_action_sequence(action_name)
        logging.info(f"机械臂动作执行结果: {result}")
        return result


def generate_action(state: RobotStatus, _observation: dict) -> dict:
    """根据当前状态生成动作"""
    action = {}
    
    if state == RobotStatus.SEARCH_TENNIS:
        action = {"type": "search", "target": "tennis"}
    elif state == RobotStatus.APPROACH_TENNIS:
        action = {"type": "approach", "target": "tennis"}
    elif state == RobotStatus.TRACK_TENNIS:
        action = {"type": "track", "target": "tennis"}
    elif state == RobotStatus.PICK:
        action = {"type": "pick"}
    elif state == RobotStatus.SEARCH_BUCKET:
        action = {"type": "search", "target": "bucket"}
    elif state == RobotStatus.TRACK_BUCKET:
        action = {"type": "track", "target": "bucket"}
    elif state == RobotStatus.PUT_BALL:
        action = {"type": "put"}
    
    return action


def main():
    """主控制循环 - 捡球循环：找球->抓球->找桶->放球->找球"""
    all_timings = []    # 每帧的处理时间
    
    robot_name = config.system.robot_id
    calibration_params = load_calibration_params(robot_name)
    logging.info(f"加载校准参数: M={calibration_params['M']:.4f}, C={calibration_params['C']:.4f}")
    
    # 初始化摄像头
    logging.info("正在初始化摄像头...")
    camera_config = config.device.hardware.camera
    camera_type = camera_config.type
    camera = CameraFactory.create_camera(camera_type, camera_config)
    
    if not camera.is_opened():
        raise IOError("无法打开摄像头")
    
    # 初始化视觉模块
    logging.info("正在初始化视觉模块...")
    vision_module = VisionModule(config)
    
    # 初始化机器人（完全通过配置文件创建）
    logging.info("正在初始化机器人...")
    robot = Robot()
    robot.camera = camera
    
    logging.info("机械臂回到初始位置...")
    robot.execute_arm_action("origin")

    # 初始化状态机
    logging.info("正在初始化状态机...")
    sm_config = {
        'target_x': config.statemachine.target_x,
        'target_distance': config.statemachine.target_distance,
        'threshold_x': config.statemachine.threshold_x,
        'threshold_d': config.statemachine.threshold_d,
        'grip_threshold': config.statemachine.grip_threshold,
        'reach_count_threshold': config.statemachine.reach_count_threshold,
        'frame_width': config.device.parameters.frame_width,
        'bucket_edge_threshold': config.statemachine.bucket_edge_threshold
    }
    state_machine = StateMachine(sm_config)
    
    # 启动 WebRTC 服务器
    if HAS_WEBRTC:
        logging.info("启动 WebRTC 服务器...")
        start_webrtc_server(port=8080)
        logging.info("WebRTC 推流地址: http://<机器人IP>:8080")
        stream_frame_interval = 1.0 / 15.0
        last_stream_time = time.time()
    else:
        logging.warning("WebRTC 依赖未安装，跳过推流功能")

    try:
        while True:
            start_time = time.time() * 1000
            
            # 捕获帧
            frame = camera.capture(flush_frames=1)
            if frame is None:
                logging.warning("无法捕获帧，重试...")
                continue
            
            height, width = frame.shape[:2]
            robot.frame_height = height
            
            # 调整帧大小
            target_width = config.device.parameters.frame_width
            if width != target_width:
                logging.warning(f"当前帧宽度 {width} 不等于 {target_width}，正在调整大小...")
                frame = cv2.resize(frame, (target_width, int(target_width * height / width)), 
                                interpolation=cv2.INTER_LINEAR)
                robot.frame_height = frame.shape[0]

            tennis_result = []
            bucket_result = []
            
            # 构建观测数据（只包含物理状态）
            observation = {
                "camera": frame,
                "timestamp": time.time(),
                "tennis_detected": False,
                "bucket_detected": False,
                "tennis_distance": float('inf'),
                "tennis_offset_x": 0.0,
                "bucket_left_edge": float('inf'),
                "bucket_right_edge": -float('inf'),
                "gripper_angle": 0
            }

            current_state = state_machine.get_state()
            
            # 网球检测
            if current_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.APPROACH_TENNIS, RobotStatus.TRACK_TENNIS]:
                tennis_result = vision_module.infer(frame)
                if tennis_result:
                    observation["tennis_detected"] = True
                    box = sorted(tennis_result, key=lambda x: x['w'], reverse=True)[0]
                    bbox_width = max(box["w"], box["h"])
                    observation["tennis_distance"] = calculate_distance(bbox_width, 
                                                                       M=calibration_params['M'],
                                                                       C=calibration_params['C'])
                    observation["tennis_offset_x"] = (box["x"] + box["w"] / 2) - target_width / 2
            
            # 桶检测
            elif current_state in [RobotStatus.SEARCH_BUCKET, RobotStatus.TRACK_BUCKET]:
                bucket_result = vision_module.get_bucket_local(frame, color="red")
                if bucket_result:
                    observation["bucket_detected"] = True
                    box = sorted(bucket_result, key=lambda x: x['w'], reverse=True)[0]
                    observation["bucket_left_edge"] = box["x"]
                    observation["bucket_right_edge"] = box["x"] + box["w"]

            # 状态机更新
            next_state = state_machine.transition(observation)
            
            if next_state != current_state:
                logging.info(f"状态转换: {current_state} -> {next_state}")
                state_machine.set_state(next_state)
                state_machine.lost_count = 0
                
                # 状态转换时的特殊处理
                if next_state == RobotStatus.PICK:
                    robot.controller.stop()
                    logging.info("开始抓取网球...")
                    result = robot.execute_arm_action("pick")
                    logging.info("抓取完成，机械臂回到初始位置...")
                    robot.execute_arm_action("origin")
                    
                    gripper_angle = result.get("gripper_angle", 0)
                    observation["gripper_angle"] = gripper_angle
                    logging.info(f"抓取动作完成，夹爪角度: {gripper_angle}")
                    
                    next_state = state_machine.transition(observation)
                    state_machine.set_state(next_state)
                    logging.info(f"抓取完成，状态转换: {RobotStatus.PICK} -> {next_state}")
                    
                    if next_state in [RobotStatus.SEARCH_BUCKET, RobotStatus.TRACK_BUCKET]:
                        robot.target_type = "bucket"
                    elif next_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS, RobotStatus.APPROACH_TENNIS]:
                        robot.target_type = "tennis"
                
                elif next_state == RobotStatus.PUT_BALL:
                    robot.controller.stop()
                    logging.info("开始放置网球...")
                    result = robot.execute_arm_action("put")
                    logging.info("放置完成，机械臂回到初始位置...")
                    robot.execute_arm_action("origin")
                    
                    next_state = state_machine.transition(observation)
                    state_machine.set_state(next_state)
                    logging.info(f"放置完成，状态转换: {RobotStatus.PUT_BALL} -> {next_state}")
                    
                    if next_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS, RobotStatus.APPROACH_TENNIS]:
                        robot.target_type = "tennis"
                
                elif next_state in [RobotStatus.SEARCH_TENNIS, RobotStatus.TRACK_TENNIS, RobotStatus.APPROACH_TENNIS]:
                    robot.target_type = "tennis"
                elif next_state in [RobotStatus.SEARCH_BUCKET, RobotStatus.TRACK_BUCKET]:
                    robot.target_type = "bucket"

            # 根据状态执行动作
            action = generate_action(state_machine.get_state(), observation)
            logging.debug(f"当前状态: {state_machine.get_state()}, 动作: {action}")

            # 执行动作
            if state_machine.get_state() in [RobotStatus.SEARCH_TENNIS, RobotStatus.SEARCH_BUCKET]:
                robot.idle()
            elif state_machine.get_state() == RobotStatus.TRACK_TENNIS:
                if observation["tennis_detected"]:
                    track_observation = {
                        "target_offset_x": observation["tennis_offset_x"],
                        "target_offset_y": 0.0,
                        "target_distance": observation["tennis_distance"]
                    }
                    command = robot.controller.track(track_observation)
                    logging.debug(f"追踪中: speed_x={command.get('x', 0):.3f}, speed_w={command.get('w', 0):.3f}")
                else:
                    robot.idle()
            elif state_machine.get_state() == RobotStatus.APPROACH_TENNIS:
                if observation["tennis_detected"]:
                    approach_observation = {
                        "target_offset_x": observation["tennis_offset_x"],
                        "target_distance": observation["tennis_distance"]
                    }
                    command = robot.controller.approach(approach_observation)
                    logging.debug(f"接近中: speed_x={command.get('x', 0):.3f}, speed_w={command.get('w', 0):.3f}")
                else:
                    robot.idle()
            elif state_machine.get_state() == RobotStatus.TRACK_BUCKET:
                if observation["bucket_detected"]:
                    bucket_center_x = (observation["bucket_left_edge"] + observation["bucket_right_edge"]) / 2
                    bucket_offset_x = bucket_center_x - config.device.parameters.frame_width / 2
                    track_observation = {
                        "target_offset_x": bucket_offset_x,
                        "target_offset_y": 0.0,
                        "target_distance": observation.get("bucket_distance", 1.0)
                    }
                    command = robot.controller.track(track_observation)
                    logging.debug(f"追踪桶: speed_x={command.get('x', 0):.3f}, speed_w={command.get('w', 0):.3f}")
                else:
                    robot.idle()
            elif state_machine.get_state() in [RobotStatus.PICK, RobotStatus.PUT_BALL]:
                robot.controller.stop()
            
            # 统计处理时间
            all_timings.append(int(time.time() * 1000 - start_time))
            avg_time = int(sum(all_timings) / len(all_timings) if all_timings else 0)
            min_time = min(all_timings) if all_timings else 0
            max_time = max(all_timings) if all_timings else 0
            logging.debug(f"当前帧处理时间: {all_timings[-1]} ms, 平均: {avg_time} ms, 最小: {min_time} ms, 最大: {max_time} ms")
            
            # WebRTC 视频推流
            if HAS_WEBRTC and video_track:
                current_time = time.time()
                if current_time - last_stream_time >= stream_frame_interval:
                    display_frame = draw_tracking_info(frame, tennis_result, bucket_result, 
                                                      state_machine.get_state(), observation)
                    video_track.push_frame(display_frame)
                    last_stream_time = current_time

    except KeyboardInterrupt:
        logging.info("收到中断信号，正在退出...")
    
    finally:
        # 释放资源
        vision_module.release()
        camera.release()
        robot.controller.stop()
        robot.controller.base.cleanup()
        if robot.arm_controller:
            robot.arm_controller.stop()
            robot.arm_controller.arm.disconnect()
        logging.info("资源已释放")

if __name__ == "__main__":
    main()
