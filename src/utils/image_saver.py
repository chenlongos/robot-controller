# src/utils/image_saver.py
"""视频帧保存工具"""

import os
import time
import logging
import cv2

# 项目根目录: src/utils/ -> src/ -> 项目根
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def save_put_picture(frame_img, bucket_dets, label, vision_module, state, observation):
    """保存带红桶检测框的视频帧到 tests/pic 目录"""
    pic_dir = os.path.join(_PROJECT_ROOT, 'tests', 'pic')
    os.makedirs(pic_dir, exist_ok=True)
    display = vision_module.draw_tracking_info(frame_img, [], bucket_dets, state, observation)
    fname = f"put_{label}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    fpath = os.path.join(pic_dir, fname)
    cv2.imwrite(fpath, display)
    logging.info(f"已保存 put {label} 帧到 {fpath}")
