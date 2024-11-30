from picamera2 import Picamera2
from aiortc import RTCPeerConnection, RTCConfiguration,RTCSessionDescription,RTCIceCandidate
from aiortc.rtcconfiguration import RTCIceServer
from aiortc.contrib.media import MediaPlayer, MediaRelay, MediaStreamTrack
from picamera2 import Picamera2
import uuid
import time
from fractions import Fraction
cam = Picamera2()
cam.configure(cam.create_video_configuration())
cam.start()


class PiCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()

    async def recv(self):
        img = cam.capture_array()

        pts = time.time() * 1000000
        new_frame = av.VideoFrame.from_ndarray(img, format='rgba')
        new_frame.pts = int(pts)
        new_frame.time_base = Fraction(1,1000000)
        return new_frame

pc = RTCPeerConnection()
cam = PiCameraTrack()
pc.addTrack(cam)