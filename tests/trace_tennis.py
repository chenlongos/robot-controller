"""网球自动追踪测试程序"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import time
import logging
import src.base
from src.controller.vision_module import VisionModule
from src.controller.base_controller import BaseController
from src.abstract.base_factory import BaseFactory
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def calculate_distance(bbox_width: int, frame_width: int, tennis_width_far: int, tennis_width_near: int) -> float:
    """根据检测框宽度估算真实世界距离（米）
    
    使用配置中的网球宽度阈值进行线性插值估算距离：
    - 当检测框宽度接近 TENNIS_WIDTH_NEAR（大）时，距离较近
    - 当检测框宽度接近 TENNIS_WIDTH_FAR（小）时，距离较远
    
    Args:
        bbox_width: 检测框宽度（像素）
        frame_width: 图像宽度（像素）
        tennis_width_far: 远处网球的参考宽度（像素）
        tennis_width_near: 近处网球的参考宽度（像素）
        
    Returns:
        estimated_distance: 估算的距离（米）
    """
    if bbox_width <= 0:
        return float('inf')
    
    if bbox_width >= tennis_width_near:
        return 0.5
    
    if bbox_width <= tennis_width_far:
        return 5.0
    
    ratio = (bbox_width - tennis_width_far) / (tennis_width_near - tennis_width_far)
    distance = 5.0 - ratio * 4.5
    
    return max(0.3, distance)


def calculate_observation(frame_width: int, frame_height: int, detections: list, config) -> dict:
    """计算追踪所需的观测数据
    
    Args:
        frame_width: 图像宽度
        frame_height: 图像高度
        detections: 检测结果列表
        config: 配置对象
        
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
    
    target_distance = calculate_distance(
        bbox_width, 
        frame_width,
        config.vision.tennis_width_far,
        config.vision.tennis_width_near
    )
    
    return {
        "target_offset_x": target_offset_x,
        "target_offset_y": target_offset_y,
        "target_distance": target_distance,
        "bbox_width": bbox_width,
        "bbox_height": bbox_height,
        "score": best_detection.get('score', 0)
    }


def draw_tracking_info(frame: cv2.Mat, detections: list, observation: dict) -> cv2.Mat:
    """在图像上绘制追踪信息"""
    frame_copy = frame.copy()
    height, width = frame_copy.shape[:2]
    
    cv2.line(frame_copy, (int(width/2), 0), (int(width/2), height), (0, 255, 0), 2)
    cv2.line(frame_copy, (0, int(height/2)), (width, int(height/2)), (0, 255, 0), 2)
    
    if detections:
        best_detection = max(detections, key=lambda d: d.get('score', 0))
        x, y, w, h = best_detection['x'], best_detection['y'], best_detection['w'], best_detection['h']
        score = best_detection.get('score', 0)
        
        cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame_copy, f"Tennis {score:.2f}", (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        cv2.circle(frame_copy, (center_x, center_y), 5, (0, 255, 0), -1)
        
        cv2.arrowedLine(frame_copy, (int(width/2), int(height/2)), 
                        (center_x, center_y), (255, 0, 0), 2)
    
    offset_x = observation.get("target_offset_x", 0)
    distance = observation.get("target_distance", float('inf'))
    
    status_text = f"Offset X: {offset_x:.1f} | Distance: {distance:.1f}"
    cv2.putText(frame_copy, status_text, (10, height - 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    mode_text = "TRACKING" if detections else "SEARCHING"
    mode_color = (0, 255, 0) if detections else (0, 165, 255)
    cv2.putText(frame_copy, mode_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
    
    return frame_copy


def main():
    """主程序入口"""
    config = load_config(robot_name="aka00v4-rk3576")
    
    camera = None
    vision = None
    base = None
    controller = None
    
    try:
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
            'search_rotation_speed': 0.5,
            'kp_angle': 0.005,
            'max_linear_speed': base_config.max_linear_speed,
            'approach_threshold': 1.5,
            'max_speed': base_config.max_linear_speed * 0.75,
            'stop_threshold': 0.5,
            'angle_threshold': 20.0,
            'approach_kp': 0.8,
            'approach_ki': 0.2,
            'approach_kd': 0.1,
            'approach_kp_angle': 0.003
        }
        
        controller = BaseController(base, controller_config)
        
        logger.info("网球追踪系统已启动")
        logger.info("按 'q' 键退出")
        
        frame_count = 0
        total_detections = 0
        start_time = time.time()
        
        STATE_SEARCH = 'search'
        STATE_TRACK = 'track'
        STATE_APPROACH = 'approach'
        STATE_STOP = 'stop'
        
        current_state = STATE_SEARCH
        search_start_time = time.time()
        
        while True:
            frame_start_time = time.time()
            
            frame = camera.capture()
            if frame is None:
                logger.warning("帧捕获失败")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            
            results = vision.infer(frame)
            
            if results:
                total_detections += len(results)
            
            height, width = frame.shape[:2]
            observation = calculate_observation(width, height, results, config)
            
            if current_state == STATE_SEARCH:
                if results:
                    current_state = STATE_TRACK
                    logger.info(f"检测到目标，切换到追踪模式")
                else:
                    command = controller.search()
                    if time.time() - search_start_time > 10:
                        search_start_time = time.time()
                        logger.info("搜索中...")
            
            elif current_state == STATE_TRACK:
                if not results:
                    current_state = STATE_SEARCH
                    search_start_time = time.time()
                    logger.info("目标丢失，切换到搜索模式")
                else:
                    if observation['target_distance'] < controller_config['approach_threshold']:
                        current_state = STATE_APPROACH
                        logger.info(f"距离 {observation['target_distance']:.1f}m，切换到接近模式")
                    else:
                        command = controller.track(observation)
                        logger.info(f"追踪中: score={observation['score']:.2f}, "
                                   f"offset_x={observation['target_offset_x']:.1f}, "
                                   f"distance={observation['target_distance']:.1f}m")
            
            elif current_state == STATE_APPROACH:
                if not results:
                    current_state = STATE_SEARCH
                    search_start_time = time.time()
                    logger.info("目标丢失，切换到搜索模式")
                else:
                    command = controller.approach(observation)
                    if command.get('done', False):
                        current_state = STATE_STOP
                        controller.stop()
                        logger.info("到达目标位置，停止运动")
                    else:
                        logger.info(f"接近中: distance={observation['target_distance']:.1f}m, "
                                   f"angle_done={command.get('angle_done', False)}, "
                                   f"distance_done={command.get('distance_done', False)}")
            
            elif current_state == STATE_STOP:
                if not results:
                    current_state = STATE_SEARCH
                    search_start_time = time.time()
                    logger.info("目标丢失，切换到搜索模式")
            
            display_frame = draw_tracking_info(frame, results, observation)
            display_frame = cv2.putText(display_frame, f"State: {current_state.upper()}", 
                                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.imshow('Tennis Tracking', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("用户按下 'q' 键，退出程序")
                break
            
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
        
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()