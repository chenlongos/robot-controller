# src/controller/vision_module.py

import cv2
import os
import logging
import numpy as np
from typing import List, Dict, Optional

class VisionModule:
    """视觉推理模块"""
    
    def __init__(self, config):
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # 模型相关
        self.session = None
        self.rknn = None
        self.input_name = None
        
        # 配置参数
        self.hardware_mode = config.device.hardware.mode
        self.model_name = config.vision.model_name
        self.model_format = config.device.hardware.model_format
        self.input_size = config.vision.input_size
        self.confidence_threshold = config.vision.confidence_threshold
        self.nms_threshold = config.vision.nms_threshold
        
        # 初始化模型
        self._init_model()
    
    def _init_model(self):
        """初始化推理模型"""
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        if self.hardware_mode == 'cpu':
            import onnxruntime as ort
            model_path = os.path.join(base_dir, 'models', f'{self.model_name}.onnx')
            self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.logger.info(f"ONNX模型已加载: {model_path}")
        
        elif self.hardware_mode == 'rk3588':
            from rknn.api import RKNN
            self.rknn = RKNN()
            model_path = os.path.join(base_dir, 'models', f'{self.model_name}.rknn')
            self.rknn.load_rknn(model_path)
            self.rknn.init_runtime(target='rk3588')
            self.logger.info(f"RKNN模型已加载: {model_path}")
        
        else:
            raise ValueError(f"不支持的硬件模式: {self.hardware_mode}")
    
    def letterbox(self, img: np.ndarray, new_shape: Optional[tuple] = None, color: tuple = (114, 114, 114)) -> np.ndarray:
        """
        YOLOv8 官方预处理函数，保持宽高比 resize + center pad
        """
        if new_shape is None:
            new_shape = (self.input_size, self.input_size)
        
        shape = img.shape[:2]  # current shape [H, W]
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw /= 2  # divide padding into 2 sides
        dh /= 2
        
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        
        return img
    
    def infer(self, frame: np.ndarray) -> List[Dict]:
        """
        执行目标检测推理
        返回检测到的目标框列表，每个框包含: x, y, w, h
        """
        if isinstance(frame, str):
            # 输入是图像路径
            orig_img = cv2.imread(frame)
            if orig_img is None:
                self.logger.warning(f"无法读取图像: {frame}")
                return []
        else:
            # 输入是视频帧（ndarray）
            orig_img = frame
        
        H, W = orig_img.shape[:2]
        input_img = self.letterbox(orig_img)
        
        # 执行推理
        if self.hardware_mode == 'cpu':
            blob = cv2.dnn.blobFromImage(input_img, scalefactor=1 / 255.0, 
                                        size=(self.input_size, self.input_size), 
                                        swapRB=True, crop=False)
            outputs = self.session.run(None, {self.input_name: blob})
            pred = outputs[0].squeeze().T  # [C, N] -> [N, C]
        
        elif self.hardware_mode == 'rk3588':
            outputs = self.rknn.inference(inputs=[input_img])
            pred = outputs[0].squeeze().T  # [C, N] -> [N, C]
        
        # 解析输出
        boxes_xywh = pred[:, :4]  # cx, cy, w, h（YOLOv8 输出是 xywh 格式）
        conf_scores = pred[:, 4]
        mask = conf_scores > self.confidence_threshold
        
        pred = pred[mask]
        boxes_xywh = boxes_xywh[mask]
        conf_scores = conf_scores[mask]
        
        # 坐标还原
        boxes = []
        raw_boxes = []
        for i in range(len(boxes_xywh)):
            cx, cy, w, h = boxes_xywh[i]
            # 使用letterbox 的 pad 参数精确还原
            shape = orig_img.shape[:2]
            r = min(self.input_size / shape[0], self.input_size / shape[1])
            new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
            dw, dh = self.input_size - new_unpad[0], self.input_size - new_unpad[1]
            dw /= 2
            dh /= 2
            
            # 将 640x640 坐标还原到缩放后尺寸（new_unpad）
            x1 = (cx - w / 2 - dw) / r
            y1 = (cy - h / 2 - dh) / r
            x2 = (cx + w / 2 - dw) / r
            y2 = (cy + h / 2 - dh) / r
            
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(W, x2)
            y2 = min(H, y2)
            
            raw_boxes.append([x1, y1, x2, y2])
        
        # NMS
        raw_boxes = np.array(raw_boxes, dtype=np.float32)
        indices = cv2.dnn.NMSBoxes(raw_boxes.tolist(), conf_scores.tolist(), 
                                    self.confidence_threshold, 
                                    self.nms_threshold)
        
        if indices is not None and len(indices) > 0:
            for idx in indices:
                i = int(idx) if np.isscalar(idx) else int(idx[0])
                x1, y1, x2, y2 = raw_boxes[i]
                box = {
                    "x": int(x1),
                    "y": int(y1),
                    "w": int(x2 - x1),
                    "h": int(y2 - y1)
                }
                boxes.append(box)
        
        return boxes
    
    def release(self):
        """释放模型资源"""
        if self.hardware_mode == 'rk3588' and self.rknn is not None:
            self.rknn.release()
            self.logger.info("RKNN模型资源已释放")