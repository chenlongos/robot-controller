"""视觉模块硬件测试 - 需要真实摄像头和RKNN模型"""
import pytest
import cv2
import time
import os
import logging
from src.controller.vision_module import VisionModule
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera

logger = logging.getLogger(__name__)


class TestVisionModule:
    """视觉模块测试类"""
    
    def test_vision_inference_with_camera(self):
        """测试从摄像头捕获帧并进行推理"""
        config = load_config(robot_name="aka00v4-lubancat3")
        
        camera = None
        vision = None
        
        try:
            camera = USBCamera({
                'device_id': config.device.hardware.camera.device_id,
                'resolution': config.device.hardware.camera.resolution,
                'frame_rate': config.device.hardware.camera.frame_rate
            })
            
            if not camera.is_opened():
                pytest.skip("摄像头未打开，跳过测试")
            
            logger.info("测试视觉模块推理功能...")
            logger.info(f"  硬件模式: {config.device.hardware.mode}")
            logger.info(f"  模型名称: {config.vision.model_name}")
            logger.info(f"  置信度阈值: {config.vision.confidence_threshold}")
            logger.info(f"  输入尺寸: {config.vision.input_size}")
            
            vision = VisionModule(config)
            
            logger.info("  等待摄像头稳定...")
            for _ in range(5):
                camera.capture()
                time.sleep(0.1)
            
            logger.info("  开始推理测试...")
            total_detections = 0
            test_frames = 10
            
            for i in range(test_frames):
                frame = camera.capture(flush_frames=1)
                if frame is None:
                    logger.warning(f"  警告: 第 {i+1} 帧捕获失败")
                    continue
                
                start_time = time.time()
                results = vision.infer(frame)
                elapsed_time = (time.time() - start_time) * 1000
                
                total_detections += len(results)
                
                logger.info(f"  帧 {i+1}/{test_frames}:")
                if results:
                    for j, box in enumerate(results):
                        x, y, w, h = box["x"], box["y"], box["w"], box["h"]
                        score = box.get("score", 0)
                        logger.info(f"    目标 {j+1}: x={x}, y={y}, w={w}, h={h}, score={score:.2f}")
                else:
                    logger.info("    未检测到目标")
                logger.info(f"    推理时间: {elapsed_time:.2f} ms")
            
            logger.info(f"  ✓ 测试完成")
            logger.info(f"  总检测帧数: {test_frames}")
            logger.info(f"  总检测目标数: {total_detections}")
            
        finally:
            if vision:
                vision.release()
            if camera:
                camera.release()
    
    def test_vision_inference_with_image(self):
        """测试使用图像文件进行推理（备用测试，无需摄像头）"""
        config = load_config(robot_name="aka00v4-lubancat3")
        
        test_image_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images', 'test.jpg')
        
        if not os.path.exists(test_image_path):
            pytest.skip(f"测试图像不存在: {test_image_path}")
        
        vision = None
        
        try:
            logger.info("测试视觉模块图像推理功能...")
            logger.info(f"  硬件模式: {config.device.hardware.mode}")
            logger.info(f"  模型名称: {config.vision.model_name}")
            
            vision = VisionModule(config)
            
            frame = cv2.imread(test_image_path)
            if frame is None:
                pytest.skip("无法读取测试图像")
            
            start_time = time.time()
            results = vision.infer(frame)
            elapsed_time = (time.time() - start_time) * 1000
            
            logger.info(f"  推理时间: {elapsed_time:.2f} ms")
            logger.info(f"  检测到 {len(results)} 个目标")
            
            if results:
                for j, box in enumerate(results):
                    x, y, w, h = box["x"], box["y"], box["w"], box["h"]
                    score = box.get("score", 0)
                    logger.info(f"    目标 {j+1}: x={x}, y={y}, w={w}, h={h}, score={score:.2f}")
            
            logger.info("  ✓ 图像推理测试完成")
            
        finally:
            if vision:
                vision.release()