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
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        
        if self.hardware_mode == 'cpu':
            import onnxruntime as ort
            model_path = os.path.join(base_dir, 'models', f'{self.model_name}.onnx')
            self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.logger.info(f"ONNX模型已加载: {model_path}")
        
        elif self.hardware_mode == 'rk3588':
            from rknn.api import RKNN
            self.rknn = RKNN()
            model_path = os.path.join(base_dir, 'models', self.model_name)
            self.rknn.load_rknn(model_path)
            self.rknn.init_runtime(target='rk3588')
            self.logger.info(f"RKNN模型已加载: {model_path}")
        
        elif self.hardware_mode == 'rk3576':
            from rknn.api import RKNN
            self.rknn = RKNN()
            model_path = os.path.join(base_dir, 'models', self.model_name)
            self.rknn.load_rknn(model_path)
            self.rknn.init_runtime(target='rk3576')
            self.logger.info(f"RKNN模型已加载: {model_path}")
        
        else:
            raise ValueError(f"不支持的硬件模式: {self.hardware_mode}")
    
    def letterbox(self, img: np.ndarray, new_shape: Optional[tuple] = None, color: Optional[tuple] = None) -> tuple:
        """
        YOLOv8 官方预处理函数，保持宽高比 resize + center pad
        返回: (处理后的图像, 缩放比例r, (dw, dh)填充偏移)
        """
        if new_shape is None:
            new_shape = (self.input_size, self.input_size)
        
        if color is None:
            if self.hardware_mode == 'rk3576':
                color = (0, 0, 0)
            else:
                color = (114, 114, 114)
        
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
        
        return img, r, (dw, dh)
    
    def dfl(self, position):
        x = position
        n, c, h, w = x.shape
        p_num = 4
        mc = c // p_num
        y = x.reshape(n, p_num, mc, h, w)
        
        exp_y = np.exp(y - np.max(y, axis=2, keepdims=True))
        y = exp_y / np.sum(exp_y, axis=2, keepdims=True)
        
        acc_metrix = np.arange(mc).astype(np.float32).reshape(1, 1, mc, 1, 1)
        y = (y * acc_metrix).sum(axis=2)
        
        return y
    
    def box_process(self, position):
        grid_h, grid_w = position.shape[2:4]
        col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
        col = col.reshape(1, 1, grid_h, grid_w)
        row = row.reshape(1, 1, grid_h, grid_w)
        grid = np.concatenate((col, row), axis=1)
        stride = np.array([self.input_size // grid_h, self.input_size // grid_w]).reshape(1, 2, 1, 1)
        
        position = self.dfl(position)
        box_xy = grid + 0.5 - position[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
        xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)
        
        return xyxy
    
    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        box_confidences = box_confidences.reshape(-1)
        _, _ = box_class_probs.shape
        
        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        
        _class_pos = np.where(class_max_score * box_confidences >= self.confidence_threshold)
        scores = (class_max_score * box_confidences)[_class_pos]
        
        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
        
        return boxes, classes, scores
    
    def nms_boxes(self, boxes, scores):
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]
        
        areas = w * h
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
            
            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1
            
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= self.nms_threshold)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep
    
    def post_process(self, input_data):
        boxes, scores, classes_conf = [], [], []
        default_branch = 3
        pair_per_branch = len(input_data) // default_branch
        for i in range(default_branch):
            boxes.append(self.box_process(input_data[pair_per_branch * i]))
            classes_conf.append(input_data[pair_per_branch * i + 1])
            scores.append(np.ones_like(input_data[pair_per_branch * i + 1][:, :1, :, :], dtype=np.float32))
        
        def sp_flatten(_in):
            ch = _in.shape[1]
            _in = _in.transpose(0, 2, 3, 1)
            return _in.reshape(-1, ch)
        
        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]
        
        boxes = np.concatenate(boxes)
        classes_conf = np.concatenate(classes_conf)
        scores = np.concatenate(scores)
        
        boxes, classes, scores = self.filter_boxes(boxes, scores, classes_conf)
        
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            b = boxes[inds]
            c = classes[inds]
            s = scores[inds]
            keep = self.nms_boxes(b, s)
            
            if len(keep) != 0:
                nboxes.append(b[keep])
                nclasses.append(c[keep])
                nscores.append(s[keep])
        
        if not nboxes and not nscores:
            return None, None, None
        
        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)
        
        return boxes, classes, scores
    
    def infer(self, frame: np.ndarray) -> List[Dict]:
        """
        执行目标检测推理
        返回检测到的目标框列表，每个框包含: x, y, w, h
        """
        if isinstance(frame, str):
            orig_img = cv2.imread(frame)
            if orig_img is None:
                self.logger.warning(f"无法读取图像: {frame}")
                return []
        else:
            orig_img = frame
        
        H, W = orig_img.shape[:2]
        input_img, r, (dw, dh) = self.letterbox(orig_img)
        
        boxes = []
        
        if self.hardware_mode == 'cpu':
            blob = cv2.dnn.blobFromImage(input_img, scalefactor=1 / 255.0, 
                                        size=(self.input_size, self.input_size), 
                                        swapRB=True, crop=False)
            outputs = self.session.run(None, {self.input_name: blob})
            pred = outputs[0].squeeze().T  # [C, N] -> [N, C]
            
            if pred.ndim != 2 or pred.shape[0] == 0:
                return []
            
            boxes_xywh = pred[:, :4]
            conf_scores = pred[:, 4]
            mask = conf_scores > self.confidence_threshold
            
            pred = pred[mask]
            boxes_xywh = boxes_xywh[mask]
            conf_scores = conf_scores[mask]
            
            raw_boxes = []
            for i in range(len(boxes_xywh)):
                cx, cy, w, h = boxes_xywh[i]
                x1 = (cx - w / 2 - dw) / r
                y1 = (cy - h / 2 - dh) / r
                x2 = (cx + w / 2 - dw) / r
                y2 = (cy + h / 2 - dh) / r
                
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)
                
                raw_boxes.append([float(x1), float(y1), float(x2 - x1), float(y2 - y1)])
            
            indices = cv2.dnn.NMSBoxes(raw_boxes, conf_scores.tolist(), 
                                        self.confidence_threshold, 
                                        self.nms_threshold)
            
            if indices is not None and len(indices) > 0:
                for idx in indices:
                    i = int(idx) if np.isscalar(idx) else int(idx[0])
                    x1, y1, w, h = raw_boxes[i]
                    box = {
                        "x": int(x1),
                        "y": int(y1),
                        "w": int(w),
                        "h": int(h),
                        "score": float(conf_scores[i])
                    }
                    boxes.append(box)
        
        elif self.hardware_mode == 'rk3588':
            outputs = self.rknn.inference(inputs=[input_img])
            pred = outputs[0].squeeze().T  # [C, N] -> [N, C]
            
            if pred.ndim != 2 or pred.shape[0] == 0:
                return []
            
            scores = pred[:, 4:]
            class_ids = np.argmax(scores, axis=1)
            conf_scores = scores[np.arange(len(scores)), class_ids]
            mask = conf_scores > self.confidence_threshold
            
            pred = pred[mask]
            conf_scores = conf_scores[mask]
            
            raw_boxes = []
            for p in pred:
                cx, cy, w, h = p[:4]
                x1 = cx - 0.5 * w
                y1 = cy - 0.5 * h
                x2 = cx + 0.5 * w
                y2 = cy + 0.5 * h
                
                x1 = (x1 - dw) / r
                y1 = (y1 - dh) / r
                x2 = (x2 - dw) / r
                y2 = (y2 - dh) / r
                
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(W, x2)
                y2 = min(H, y2)
                
                raw_boxes.append([float(x1), float(y1), float(x2 - x1), float(y2 - y1)])
            
            indices = cv2.dnn.NMSBoxes(raw_boxes, conf_scores.tolist(), 
                                        self.confidence_threshold, 
                                        self.nms_threshold)
            
            if indices is not None and len(indices) > 0:
                for idx in indices:
                    i = int(idx) if np.isscalar(idx) else int(idx[0])
                    x1, y1, w, h = raw_boxes[i]
                    box = {
                        "x": int(x1),
                        "y": int(y1),
                        "w": int(w),
                        "h": int(h),
                        "score": float(conf_scores[i])
                    }
                    boxes.append(box)
        
        elif self.hardware_mode == 'rk3576':
            input_img_rgb = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
            outputs = self.rknn.inference(inputs=[input_img_rgb], data_format='nhwc')
            
            det_boxes, _, det_scores = self.post_process(outputs)
            
            if det_boxes is not None:
                for i in range(det_boxes.shape[0]):
                    x1, y1, x2, y2 = det_boxes[i]
                    
                    x1 = (x1 - dw) / r
                    y1 = (y1 - dh) / r
                    x2 = (x2 - dw) / r
                    y2 = (y2 - dh) / r
                    
                    x1 = max(0, x1)
                    y1 = max(0, y1)
                    x2 = min(W, x2)
                    y2 = min(H, y2)
                    
                    box = {
                        "x": int(x1),
                        "y": int(y1),
                        "w": int(x2 - x1),
                        "h": int(y2 - y1),
                        "score": float(det_scores[i])
                    }
                    boxes.append(box)
        
        return boxes
    
    def release(self):
        """释放模型资源"""
        if (self.hardware_mode == 'rk3588' or self.hardware_mode == 'rk3576') and self.rknn is not None:
            self.rknn.release()
            self.logger.info("RKNN模型资源已释放")