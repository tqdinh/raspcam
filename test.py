import socketio
import asyncio
from aiortc import RTCPeerConnection, RTCConfiguration,RTCSessionDescription,RTCIceCandidate
from aiortc.rtcconfiguration import RTCIceServer
from aiortc.contrib.media import MediaPlayer,MediaStreamTrack
from aiortc.sdp import candidate_from_sdp, candidate_to_sdp
from typing import Dict, Optional, Set, Union
import av
from av.frame import Frame
from av.packet import Packet

from picamera2 import Picamera2
import time
from fractions import Fraction
cam = Picamera2()
cam.configure(cam.create_video_configuration())
cam.start()

class PiCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()

    async def recv(self) -> Union[Frame, Packet]:
        # print(type (Frame))
        # print(type (Packet))
        print("call rcv----->")
        img = cam.capture_array()
        pts = time.time() * 1000000
        new_frame = av.VideoFrame.from_ndarray(img, format='rgba')
        new_frame.pts = int(pts)
        new_frame.time_base = Fraction(1,1000000)
        return new_frame

# Create a Socket.IO client
sio = socketio.AsyncClient()
stunServer =RTCIceServer(urls="stun:stun.l.google.com:19302")
turnServer =RTCIceServer(urls= "turn:openrelay.mextered.ca:443",username ="openrelayproject",credential ="openrelayproject")
iceServers =[stunServer,turnServer]

rtcConfig =RTCConfiguration(iceServers=iceServers)
pc = None
user= "respberry"
room="123"
peerReady=False

player = MediaPlayer(
            'http://download.tsi.telecom-paristech.fr/'
            'gpac/dataset/dash/uhd/mux_sources/hevcds_720p30_2M.mp4')

# player =MediaPlayer('/dev/video0',format="v4l2", options={
#             'video_size': '640x480'})

# Event: Connection
@sio.event
async def connect():
    print("Connected to the server")
    # Join a room
    await sio.emit("join", { "username": user, "room": room })
    print("Joined room: test_room")

# Event: Receive room messages

def getDataType(data:dict):
    if data['type'] in ['answer','offer']:
        return RTCSessionDescription(sdp=data['sdp'],type=data['type'])
    elif data['type'] == 'candidate' and data['candidate']:
        dictCandidate :dict=data['candidate']
        candidate = candidate_from_sdp( dictCandidate['candidate'].split(":",1)[1])
        candidate.sdpMid = dictCandidate["sdpMid"]
        candidate.sdpMLineIndex = dictCandidate["sdpMLineIndex"]
        return candidate

@sio.on("data")
async def on_room_message(data:dict):
    global sio
    global user
    global room
    global pc
    global player
    while True:
        obj =getDataType(data)
        if isinstance(obj,RTCSessionDescription):

            await pc.setRemoteDescription(obj)
            if 'offer' == obj.type:

                #pc.addTrack(player.video)
                
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
    
                rtc_dict = {"type": pc.localDescription.type,"sdp": pc.localDescription.sdp}
                await sio.emit("data",{ "username": user, "room": room ,"data":rtc_dict})

        elif isinstance(obj,RTCIceCandidate):
            await pc.addIceCandidate(obj)
       
        break



@sio.on("ready")
async def on_room_message(data):
    print("on Ready:", data)
    global peerReady
    global sio
    global user
    global room
    global pc
    if False == peerReady:
        peerReady=True
        await sio.emit("ready", { "username": user, "room": room })

        pc = RTCPeerConnection()

        mycam = PiCameraTrack()
       
        pc.addTrack(mycam)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)

        @pc.on("track")
        async def on_track(track):
            print(f"Track {track.kind} received")

        @pc.on("icecandidate")
        async def onicecandidate(candidate):
            print("ICECANDIATE state is %s" % candidate)
           
            

        



# Send a message to the room
async def send_message():
    await sio.emit("send_room_message", {"message": "Hello, Room!"})

# Event: Disconnection
@sio.event
async def disconnect():
    print("Disconnected from the server")

async def main():
    # Connect to the server
    
    await sio.connect('wss://robortc.el.r.appspot.com/')

    # Send a test message
    await send_message()

    # Wait to keep the connection alive
    await sio.wait()

if __name__ == "__main__":
    asyncio.run(main())
