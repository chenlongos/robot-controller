"""视觉模块硬件测试 - 需要真实摄像头和RKNN模型"""
import sys
import os
import time
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
from src.controller.vision_module import VisionModule
from src.config_loader import load_config
from src.camera.usb_camera import USBCamera
from _robot_select import select_robot

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def test_vision_inference_with_camera(config):
    """测试从摄像头捕获帧并进行推理"""
    camera = None
    vision = None

    try:
        camera = USBCamera({
            'device_id': config.device.hardware.camera.device_id,
            'resolution': config.device.hardware.camera.resolution,
            'frame_rate': config.device.hardware.camera.frame_rate
        })

        if not camera.is_opened():
            print("摄像头未打开，跳过测试")
            return

        logger.info("测试视觉模块推理功能...")
        logger.info(f"  硬件模式: {config.device.hardware.mode}")
        logger.info(f"  模型名称: {config.vision.model_name}")
        logger.info(f"  置信度阈值: {config.vision.confidence_threshold}")
        logger.info(f"  输入尺寸: {config.device.hardware.camera.resolution[0]}")

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


def test_vision_inference_with_image(config):
    """测试使用图像文件进行推理（备用测试，无需摄像头）"""
    test_image_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images', 'test.jpg')

    if not os.path.exists(test_image_path):
        print(f"测试图像不存在: {test_image_path}，跳过测试")
        return

    vision = None

    try:
        logger.info("测试视觉模块图像推理功能...")
        logger.info(f"  硬件模式: {config.device.hardware.mode}")
        logger.info(f"  模型名称: {config.vision.model_name}")

        vision = VisionModule(config)

        frame = cv2.imread(test_image_path)
        if frame is None:
            print("无法读取测试图像，跳过测试")
            return

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


def select_test_type():
    """交互式选择测试类型"""
    print("\n请选择测试类型:")
    print("  1. 摄像头推理测试")
    print("  2. 图像文件推理测试")
    print("  3. 全部测试")

    while True:
        try:
            choice = input("\n请选择 (1-3，默认3): ").strip()
            if choice == "" or choice == "3":
                return "all"
            elif choice == "1":
                return "camera"
            elif choice == "2":
                return "image"
            else:
                print("请输入 1、2 或 3")
        except (EOFError, KeyboardInterrupt):
            print("\n已取消")
            return None


def main():
    robot_name = select_robot()
    if robot_name is None:
        return 0

    test_type = select_test_type()
    if test_type is None:
        return 0

    print(f"\n正在加载 {robot_name} 的配置...")
    config = load_config(robot_name=robot_name)

    try:
        if test_type in ("camera", "all"):
            test_vision_inference_with_camera(config)
        if test_type in ("image", "all"):
            test_vision_inference_with_image(config)
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
