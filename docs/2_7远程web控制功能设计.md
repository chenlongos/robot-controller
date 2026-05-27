## 远程Web控制功能设计

### 1 功能概述

设计一个远程Web控制界面，允许用户在浏览器中实时监控和控制机器人。

**核心功能点：**

| 功能模块 | 说明 |
|----------|------|
| **实时状态监控** | 摄像头画面、机械臂状态、底盘状态实时显示 |
| **远程控制** | 启动/停止机器人运行 |
| **配置实时修改** | 动态调整机械臂动作序列、底盘运动参数 |
| **配置保存** | 将修改后的配置保存为新的配置文件 |
| **校准文件管理** | 校准文件的制作、保存与加载 |

### 2架构设计

┌─────────────────────────────────────────────────────────────┐
│ Web 客户端 (Browser) │ │ 
 │ │ │ 视频监控面板 │ │ 状态显示面板 │ │ 配置编辑面板 │ │ │ 
 └──────────────┘ └──────────────┘ └──────────────┘ │ 
 └───────────────────────────┬─────────────────────────────────┘
  │ WebSocket / HTTP ▼ 
  ┌─────────────────────────────────────────────────────────────┐
  │ Web 服务器 (FastAPI) │ │ 
   ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ 
   │ │ │ 视频流服务 │ │ 状态API │ │ 配置管理API │ │ │ 
   └──────────────┘ └──────────────┘ └──────────────┘ │
  └───────────────────────────┬─────────────────────────────────┘
     │ 内部接口 ▼ 
     ┌─────────────────────────────────────────────────────────────┐
      │ 机器人控制系统 │ │ 
      ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
       │ │ │ 摄像头驱动 │ │ 状态管理器 │ │ 配置管理器 │ │ │ 
       └──────────────┘ └──────────────┘ └──────────────┘ │ └─────────────────────────────────────────────────────────────┘


### 3 模块划分

src/ 
├── ... (原有模块)
└── web/ # Web服务模块
  ├── init.py
  ├── server.py # FastAPI服务器入口
  ├── routers/ # API路由
  │ ├── status.py # 状态API
  │ ├── control.py # 控制API
  │ ├── config.py # 配置API
  │ └── calibration.py # 校准文件API
  ├── websocket/ # WebSocket服务
  │ └── video_stream.py # 视频流推送
  └── static/ # 前端静态文件
    ├── index.html
    ├── css/
    └── js/


### 4 API接口设计

#### 状态查询接口

| API路径 | HTTP方法 | 功能描述 |
|---------|----------|----------|
| `/api/status` | GET | 获取机器人当前状态 |
| `/api/status/arm` | GET | 获取机械臂状态 |
| `/api/status/base` | GET | 获取底盘状态 |
| `/api/status/camera` | GET | 获取摄像头状态 |

**响应格式示例：**

```json
{
    "robot_status": "search",
    "timestamp": 1620000000.0,
    "arm": {
        "joint_positions": [90.0, 45.0, 30.0, 0.0, 90.0, 0.0],
        "gripper_position": 0.5,
        "is_moving": false
    },
    "base": {
        "velocity": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
        "is_moving": false
    },
    "camera": {
        "resolution": [640, 480],
        "fps": 30,
        "is_streaming": true
    }
}
```

#### 控制指令接口

| API路径 | HTTP方法 | 功能描述 |
|---------|----------|----------|
| `/api/control/start` | POST | 启动机器人 |
| `/api/control/stop` | POST | 停止机器人 |
| `/api/control/emergency_stop` | POST | 紧急停止 |

**请求格式示例：**

```json
{
    "command": "start",
    "mode": "auto"  // auto: 自动模式, manual: 手动模式
}
```

#### 配置管理接口

| API路径 | HTTP方法 | 功能描述 |
|---------|----------|----------|
| `/api/config` | GET | 获取当前配置 |
| `/api/config` | PUT | 更新配置（运行时生效） |
| `/api/config/save` | POST | 保存配置为新文件 |
| `/api/config/list` | GET | 列出所有配置文件 |
| `/api/config/{name}` | GET | 获取指定配置文件 |

#### 校准文件管理接口

| API路径 | HTTP方法 | 功能描述 |
|---------|----------|----------|
| `/api/calibration` | GET | 获取当前校准文件列表 |
| `/api/calibration/upload` | POST | 上传校准文件 |
| `/api/calibration/download/{name}` | GET | 下载指定校准文件 |
| `/api/calibration/delete/{name}` | DELETE | 删除指定校准文件 |
| `/api/calibration/create` | POST | 创建新的校准文件 |
| `/api/calibration/{name}` | GET | 获取指定校准文件内容 |

**配置更新请求格式：**

```json
{
    "arm": {
        "pick_sequence": [
            {"joints": [90, 90, 90, 0, 90, 0], "duration": 1.0},
            {"joints": [90, 60, 30, 0, 90, 0], "duration": 1.0},
            {"joints": [90, 60, 30, 0, 90, 0], "duration": 0.5, "gripper": 0.0},
            {"joints": [90, 90, 90, 0, 90, 0], "duration": 1.0}
        ],
        "max_joint_speed": 30.0
    },
    "base": {
        "max_speed": 0.5,
        "kp": 0.8,
        "ki": 0.0,
        "kd": 0.1
    }
}
```

**配置保存请求格式：**

```json
{
    "filename": "robot_custom.yaml",
    "config": {...}
}
```

### 5 WebSocket视频流

```python
# src/web/websocket/video_stream.py
import cv2
import base64
from fastapi import WebSocket
from typing import List

class VideoStreamManager:
    """视频流管理器"""
    
    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []
        self.running: bool = False
        self.camera = None
    
    async def connect(self, websocket: WebSocket) -> None:
        """接受新连接"""
        pass
    
    def disconnect(self, websocket: WebSocket) -> None:
        """断开连接"""
        pass
    
    async def start_streaming(self) -> None:
        """持续推送视频帧"""
        pass
```

### 12.6 前端界面设计

#### 12.6.1 界面布局

┌─────────────────────────────────────────────────────────────┐
│ 标题栏: 机器人远程控制中心 │ 
├──────────────┬──────────────────────────────────────────────┤
 │ │ │ │ 左侧面板 │ 主显示区 │ │ ┌────────┐ │ 
 ┌─────────────────────────────────────┐ 
 │ │ │ 状态 │ │ │ 摄像头实时画面 │ │ │ │ 监控 
 │ │ │ │ │ │ ├────────┤ │ │ │ │ │ │ 配置 │ │ │ │ │ │ │ 编辑 │ │ 
 └─────────────────────────────────────┘
  │ │ ├────────┤ │ │ │ │ 配置 │ │ ┌
  ─────────────────────────────────────┐
   │ │ │ 管理 │ │ │ 控制按钮: [启动] [停止] [紧急停止] │ │ │ └────────┘ │ 
   └─────────────────────────────────────┘ 
   │ │ │ │ 
   └──────────────┴──────────────────────────────────────────────┘

#### 12.6.2 状态监控面板

| 监控项 | 显示内容 |
|--------|----------|
| 机器人状态 | 当前状态（INIT/SEARCH/TRACKING/PICK等） |
| 机械臂关节 | 6个关节角度实时数值 |
| 夹爪状态 | 夹爪开合度（0-100%） |
| 底盘速度 | X/Y/旋转速度 |
| 运行时长 | 自启动以来的运行时间 |
| 任务进度 | 已完成的捡球放桶次数 |

#### 12.6.3 配置编辑面板

**机械臂配置：**
- 抓取动作序列（关节角度数组 + 持续时间）
- 放置动作序列
- 最大关节速度
- 夹爪开合速度

**底盘配置：**
- 最大线速度
- 最大角速度
- PID参数（Kp, Ki, Kd）
- 距离容差
- 角度容差

#### 12.6.4 配置管理面板

- 配置文件列表展示
- 选择加载配置文件
- 保存当前配置为新文件
- 删除配置文件（需确认）

### 12.7 关键实现

#### 12.7.1 Web服务器启动

```python
# src/web/server.py

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from .routers import status, control, config, calibration
from .websocket.video_stream import VideoStreamManager

app = FastAPI(title="机器人远程控制中心")

# 注册路由
app.include_router(status.router, prefix="/api/status")
app.include_router(control.router, prefix="/api/control")
app.include_router(config.router, prefix="/api/config")
app.include_router(calibration.router, prefix="/api/calibration")

# 静态文件服务
app.mount("/static", StaticFiles(directory="src/web/static"), name="static")

# 视频流管理器
video_manager = VideoStreamManager()

def setup_routes() -> None:
    """设置路由"""
    pass

def setup_static_files() -> None:
    """设置静态文件服务"""
    pass

def start_server(host: str = "0.0.0.0", port: int = 8000) -> None:
    """启动Web服务器"""
    pass
```

#### 12.7.2 视频流推送

```python
# src/web/websocket/video_stream.py

import cv2
import base64
import asyncio
from fastapi import WebSocket

class VideoStreamManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
        self.running = False
        self.camera = None
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        if not self.running:
            self.running = True
            await self.start_streaming()
    
    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        if not self.active_connections:
            self.running = False
    
    async def start_streaming(self):
        """持续推送视频帧"""
        while self.running:
            if self.camera is None:
                # 获取摄像头实例
                from src.cameras.usb_camera import USBCamera
                self.camera = USBCamera.get_instance()
            
            frame = self.camera.capture()
            if frame is not None:
                # 编码为JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                frame_base64 = base64.b64encode(buffer).decode('utf-8')
                
                # 发送给所有连接的客户端
                for connection in self.active_connections:
                    try:
                        await connection.send_json({
                            "type": "frame",
                            "data": frame_base64,
                            "timestamp": time.time(),
                            "width": frame.shape[1],
                            "height": frame.shape[0]
                        })
                    except Exception:
                        self.active_connections.remove(connection)
            
            await asyncio.sleep(0.033)  # ~30fps
```

#### 12.7.3 状态API

```python
# src/web/routers/status.py

from fastapi import APIRouter
from src.state_machine import state_machine
from src.factories.base_factory import BaseFactory
from src.factories.arm_factory import ArmFactory

router = APIRouter()

@router.get("/")
async def get_status():
    """获取机器人完整状态"""
    arm = ArmFactory.get_instance()
    base = BaseFactory.get_instance()
    
    return {
        "robot_status": state_machine.get_state().value,
        "timestamp": time.time(),
        "arm": {
            "joint_positions": arm.get_joint_positions(),
            "gripper_position": arm.get_gripper_position(),
            "is_moving": arm.is_moving()
        },
        "base": {
            "velocity": base.get_velocity(),
            "position": base.get_position(),
            "is_moving": base.is_moving()
        }
    }

@router.get("/arm")
async def get_arm_status():
    """获取机械臂状态"""
    pass

@router.get("/base")
async def get_base_status():
    """获取底盘状态"""
    pass

@router.get("/camera")
async def get_camera_status():
    """获取摄像头状态"""
    pass
```

#### 12.7.4 配置管理API

```python
# src/web/routers/config.py

from fastapi import APIRouter, HTTPException
from src.config_loader import ConfigLoader, Config
import yaml
import os

router = APIRouter()
config_loader = ConfigLoader()

@router.get("/")
async def get_current_config():
    """获取当前配置"""
    return config_loader.get_current_config().dict()

@router.put("/")
async def update_config(config_updates: dict):
    """更新配置（运行时生效）"""
    try:
        config_loader.update_config(config_updates)
        return {"status": "success", "message": "配置已更新"}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/save")
async def save_config(request: dict):
    """保存配置为新文件"""
    filename = request.get("filename")
    config_data = request.get("config")
    
    if not filename:
        raise HTTPException(status_code=400, detail="缺少文件名")
    
    save_path = f"config/devices/{filename}"
    try:
        with open(save_path, 'w') as f:
            yaml.dump(config_data, f, default_flow_style=False)
        return {"status": "success", "path": save_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/list")
async def list_configs():
    """列出所有配置文件"""
    config_dir = "config/devices/"
    files = [f for f in os.listdir(config_dir) if f.endswith('.yaml')]
    return {"configs": files}

@router.get("/{name}")
async def get_config(name: str):
    """获取指定配置文件"""
    pass

#### 校准文件管理API

```python
# src/web/routers/calibration.py

from fastapi import APIRouter, HTTPException, UploadFile, File
from typing import List
import os
import yaml

router = APIRouter()
CALIBRATION_DIR = "calibration/"

@router.get("/")
async def list_calibrations():
    """列出所有校准文件"""
    if not os.path.exists(CALIBRATION_DIR):
        os.makedirs(CALIBRATION_DIR)
    
    files = [f for f in os.listdir(CALIBRATION_DIR) if f.endswith('.yaml') or f.endswith('.json')]
    return {"calibrations": files}

@router.post("/upload")
async def upload_calibration(file: UploadFile = File(...)):
    """上传校准文件"""
    if not os.path.exists(CALIBRATION_DIR):
        os.makedirs(CALIBRATION_DIR)
    
    file_path = os.path.join(CALIBRATION_DIR, file.filename)
    
    try:
        with open(file_path, 'wb') as f:
            content = await file.read()
            f.write(content)
        return {"status": "success", "path": file_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/download/{name}")
async def download_calibration(name: str):
    """下载指定校准文件"""
    file_path = os.path.join(CALIBRATION_DIR, name)
    
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="文件不存在")
    
    return FileResponse(file_path)

@router.delete("/delete/{name}")
async def delete_calibration(name: str):
    """删除指定校准文件"""
    file_path = os.path.join(CALIBRATION_DIR, name)
    
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="文件不存在")
    
    try:
        os.remove(file_path)
        return {"status": "success", "message": f"已删除 {name}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/create")
async def create_calibration(request: dict):
    """创建新的校准文件"""
    filename = request.get("filename")
    data = request.get("data")
    
    if not filename:
        raise HTTPException(status_code=400, detail="缺少文件名")
    
    if not data:
        raise HTTPException(status_code=400, detail="缺少数据")
    
    if not os.path.exists(CALIBRATION_DIR):
        os.makedirs(CALIBRATION_DIR)
    
    file_path = os.path.join(CALIBRATION_DIR, filename)
    
    try:
        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        return {"status": "success", "path": file_path}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/{name}")
async def get_calibration(name: str):
    """获取指定校准文件内容"""
    file_path = os.path.join(CALIBRATION_DIR, name)
    
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="文件不存在")
    
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        return data
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```
```

#### 12.7.5 控制指令API
```python
# src/web/routers/control.py
from fastapi import APIRouter
from typing import Dict

router: APIRouter = APIRouter()

@router.post("/start")
async def start_robot(command: Dict[str, str]):
    """启动机器人"""
    pass

@router.post("/stop")
async def stop_robot():
    """停止机器人"""
    pass

@router.post("/emergency_stop")
async def emergency_stop():
    """紧急停止"""
    pass
```


### 12.8 安全考虑

| 安全措施 | 说明 |
|----------|------|
| **访问控制** | 支持用户名密码认证 |
| **WebSocket认证** | 连接前需验证Token |
| **请求频率限制** | 防止API滥用 |
| **紧急停止优先级** | 紧急停止指令最高优先级 |
| **配置校验** | 修改配置前进行参数合法性校验 |
| **日志记录** | 记录所有控制指令和配置修改 |

### 12.9 部署方案

```bash
# 安装依赖
pip install fastapi uvicorn opencv-python websockets

# 启动Web服务器
uvicorn src.web.server:app --host 0.0.0.0 --port 8000 --reload

# 访问地址
# http://<robot_ip>:8000/static/index.html
```

### 12.10 功能优势

| 特性 | 说明 |
|------|------|
| **实时监控** | WebSocket实时推送视频流和状态数据 |
| **远程控制** | 支持远程启动/停止机器人 |
| **动态配置** | 运行时修改配置参数，无需重启 |
| **配置持久化** | 支持保存配置为新文件 |
| **跨平台访问** | 任何支持浏览器的设备均可访问 |
| **响应式设计** | 适配桌面和移动设备 |