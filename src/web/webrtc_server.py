# src/web/webrtc_server.py
"""WebRTC视频推流模块"""

import logging
import asyncio
import threading
import json
import fractions

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


class OpenCVVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.frame_queue = asyncio.Queue(maxsize=5)
        self._start_time = 0
        self._frame_count = 0

    async def recv(self):
        frame = await self.frame_queue.get()
        return frame

    def push_frame(self, cv_frame):
        self._frame_count += 1
        
        frame = VideoFrame.from_ndarray(cv_frame, format="bgr24")
        frame.pts = int(self._frame_count * 33333)
        frame.time_base = fractions.Fraction(1, 30000)
        
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


async def on_shutdown(app: web.Application) -> None:
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def start_webrtc_server(port: int = 8080):
    if not HAS_WEBRTC:
        logging.warning("WebRTC 依赖未安装，跳过推流功能")
        return

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


def push_frame(frame):
    global video_track
    if video_track is not None:
        video_track.push_frame(frame)


def is_available():
    return HAS_WEBRTC