from abc import abstractmethod
import asyncio
import socketio
from picamera2 import Picamera2
import time
from fractions import Fraction
from aiortc.contrib.media import MediaStreamTrack
from aiortc.sdp import candidate_from_sdp
from aiortc import RTCPeerConnection, RTCConfiguration,RTCSessionDescription,RTCIceCandidate
from aiortc.rtcconfiguration import RTCIceServer
from typing import Union
import av 
from av.frame import Frame
from av.packet import Packet
import json
import time
import math
import serial

STUN_SERVER="stun:stun.l.google.com:19302"
SIGNAL_SERVER="wss://robortc.el.r.appspot.com/"
COMMAND_CHANNEL ="commands"

class PiCameraTrack(MediaStreamTrack):
    kind = "video"
    cam = Picamera2()
    def __init__(self):
        super().__init__()
        
        cam_config =self.cam.create_video_configuration( {"size":(640,480)},controls={"FrameRate": 15})
        self.cam.configure(cam_config)
        self.cam.start()

    def startToCapture(self):
        self.cam.start()
    def stopCapture(self):
        self.cam.stop

    async def recv(self) -> Union[Frame, Packet]:
      
        img = self.cam.capture_array()
        pts = time.time() * 1000000
        new_frame = av.VideoFrame.from_ndarray(img, format='rgba')
        new_frame.pts = int(pts)
        new_frame.time_base = Fraction(1,1000000)
        return new_frame

class Signal:
    @abstractmethod
    def send(self,data):pass
    @abstractmethod
    def setOnListenConnected(self, onConnected):pass
    @abstractmethod
    def setOnListenClosed(self,onClosed ):pass
    @abstractmethod
    def setOnListenData(self, onData):pass
    @abstractmethod
    def setOnListenReady(self, onReady):pass
    @abstractmethod
    def initConnection(self):pass
    @abstractmethod
    def emit(self, event , data):pass



class SocketIOSignal(Signal):
    sioDomain:str =""
    onConnected =None
    onDisconnected=None
    onData=None
    onReady=None
    def __init__(self,sioDomain,room,username):
        super().__init__()
        self.sioDomain=sioDomain
        self.sio  = socketio.AsyncClient()
        self.username =username
        self.room=room

        @self.sio.event
        async def connect():
            if self.onConnected:
                self.onConnected()

            print("Connected to the server")
            # Join a room
            await self.sio.emit("join", { "username": self.username, "room": self.room })
            print("Joined room: {self.room} then room auto send ready to the others")

        @self.sio.event
        async def disconnect():
            if self.disconnect:
                self.disconnect()
            print("close connection to the server")

        @self.sio.on("data")
        async def on_room_message(data:dict):
            if self.onData:
                await self.onData(data)

        @self.sio.on("ready")
        async def on_room_message_ready(data):
            if self.onReady:
                await self.onReady(data)
 
    async def emit(self, event, data):
        await self.sio.emit(event,data)
    

    async def initConnection(self):
        # pass
        await self.sio.connect(self.sioDomain)
        await self.sio.wait()

    def setOnListenConnected(self, onConnected):
        self.onConnected =onConnected
    
    def setOnListenClosed(self,onDisconnected):
        self.disconnect=onDisconnected

    def setOnListenReady(self, onReady):
        self.onReady = onReady

    def setOnListenData(self, onData):
        self.onData = onData

   
class RobotRTC:
    signal : Signal =None
    def __init__(self,signal :Signal,cameraVideo:PiCameraTrack,serialCommunication:serial):
        self.signal =signal
        self.cameraVideo =cameraVideo
        self.audio = None
        self.serialCommunication =serialCommunication
        self.rtcConfig =RTCConfiguration(iceServers=[RTCIceServer(urls=STUN_SERVER)
                                        # ,RTCIceServer(urls= "turn:openrelay.mextered.ca:443"
                                        #               ,username ="openrelayproject"
                                        #               ,credential ="openrelayproject")
                                                      ])
        self.peerConnection =None
        self.dataChannel =None
        
        self.user= "respberry"
        self.room="123"

        self.peerReady=False
        self.timeStamp = int(time.time() * 1000)
        self.deltaTime=0
        self.numberOfCommandAllowPersecond =5
        self.cps =1000.0/self.numberOfCommandAllowPersecond # 10 comand /1000 ms 
       
       

    
        signal.setOnListenConnected( lambda : {
             print("RoboRTC connected")
        })

        signal.setOnListenClosed( lambda :{print("RoboRTC CLOSED")}
            
        )

        signal.setOnListenReady(lambda onReady: self.onPeerReady(onReady))

        signal.setOnListenData(lambda onData :self.onPeerData(onData))
    
    def listenDataChannel(self):

        @self.peerConnection.on("datachannel")
        def on_datachannel(channel):
            @channel.on("message")
            def on_message(message):
                print(channel, "<", message)
                if isinstance(message, str) and message.startswith("ping"):
                    # reply
                    print(channel, "pong" + message[4:])


        @self.dataChannel.on("bufferedamountlow")
        def on_bufferedamountlow():
            print("Buffered amount is low, ready to send more data.")
            
        @self.dataChannel.on("open")
        def on_open():
            print("Data channel is open")
            self.dataChannel.send("Hello, peer!  --------------------")

        @self.dataChannel.on("close")
        async def on_close():
            print("Data channel is closed")
            await self.onCleanupResource()

        # Handle the 'error' event (if there is an error with the data channel)
        @self.dataChannel.on("error")
        def on_error(error):
            print("Error on data channel: %s", error)

        @self.dataChannel.on("message")
        def on_message(message):        
            while True:

                self.deltaTime = int(time.time()*1000) - self.timeStamp
                if self.deltaTime >= self.cps:
                    # period between 2 command longer than the limit
                    # take timestamp at this time
                    # otherwise not take the timestamp
                    self.timeStamp=int(time.time()*1000)
                else:
                    try:
                        #{"joystick":"LEFT","x":0.0,"y":0.0}
                        data = json.loads(message)
                        x=data['x']
                        y=data['y']
                        if x!=0.0 or y!=0.0:
                            # this is not the termial message #{"joystick":"LEFT","x":0.0,"y":0.0}#
                            break      
                        # else:
                        #     print("-------keep---------" )
                    except json.JSONDecodeError as e:
                        print("Invalid JSON:", e)
            
                try:
                    data = json.loads(message)
                    x=data['x']
                    y=data['y']
                    radian =-1*math.atan2(y,x)
                    debugCommandMessge(radian)

                except json.JSONDecodeError as e:
                    print("Invalid JSON:", e)

                if None != self.serialCommunication:
                    self.serialCommunication.write( f"<{message}>".encode('ascii'))
                    self.serialCommunication.flush()
                else:
                    print("No Arduino device")

                break

    async def onPeerReady(self,peer):
        print(f"on PeerReady: {peer}")
        if False == self.peerReady:
            self.peerReady =True
            await self.signal.emit("ready", { "username": self.user, "room": self.room })

            self.peerConnection =  RTCPeerConnection(configuration= self.rtcConfig)
            self.dataChannel = self.peerConnection.createDataChannel(COMMAND_CHANNEL,ordered = True,maxRetransmits=10)
            self.listenDataChannel()       
            self.peerConnection.addTrack(self.cameraVideo)
      
    async def onPeerData(self,data):
        if None == self.peerConnection:
            print("Peer not created")
            return 
        while True:
            obj =self.getDataType(data)
            if isinstance(obj,RTCSessionDescription):

                await self.peerConnection.setRemoteDescription(obj)
                if 'offer' == obj.type:

                    #pc.addTrack(player.video)
                    
                    answer = await self.peerConnection.createAnswer()
                    await self.peerConnection.setLocalDescription(answer)
        
                    rtc_dict = {"type": self.peerConnection.localDescription.type,"sdp": self.peerConnection.localDescription.sdp}
                    await self.signal.emit("data",{ "username": self.user, "room": self.room ,"data":rtc_dict})

            elif isinstance(obj,RTCIceCandidate):
                await self.peerConnection.addIceCandidate(obj)
        
            break

    def getDataType(self,data:dict):
        if data['type'] in ['answer','offer']:
            return RTCSessionDescription(sdp=data['sdp'],type=data['type'])
        elif data['type'] == 'candidate' and data['candidate']:
            dictCandidate :dict=data['candidate']
            candidate = candidate_from_sdp( dictCandidate['candidate'].split(":",1)[1])
            candidate.sdpMid = dictCandidate["sdpMid"]
            candidate.sdpMLineIndex = dictCandidate["sdpMLineIndex"]
            return candidate

    async def start(self):
        await self.signal.initConnection()

    async def onCleanupResource(self):
        self.cameraVideo.startToCapture()
        self.dataChannel.close()
        await self.peerConnection.close()
        self.peerReady =False

def debugCommandMessge(rawArcRadian, isShow :bool=False):
    if not isShow:
        return
        
    exceptArc =math.pi/8
    joystickRadian =rawArcRadian
    print("Radian: ",joystickRadian )
    print("Radian upper : ", math.pi - exceptArc  )
    print("Radian lower : ",    -1* math.pi + exceptArc  )
    while True:
        
        if (rawArcRadian < exceptArc) and (rawArcRadian> -1 * exceptArc):
            joystickRadian =0
            print("0pi-------------------")
            break
        
        if (rawArcRadian > math.pi/2 - exceptArc) and (rawArcRadian < math.pi/2 + exceptArc):
            joystickRadian= math.pi/2
            print("pi/2------------------")
            break

        if (rawArcRadian > math.pi - exceptArc  and rawArcRadian < math.pi) or (rawArcRadian < -1* math.pi + exceptArc and rawArcRadian>-1* math.pi):
            joystickRadian = math.pi
            print("pi ------------")
            break

        if( rawArcRadian < -1 * math.pi/2 + exceptArc) and (rawArcRadian > -1 * math.pi/2 - exceptArc):
            joystickRadian = -1* math.pi/2
            print("-pi ----------------")
            break
        break
    
    
    backAndForth = math.sin(joystickRadian)
    leftAndRight = math.cos(joystickRadian)
    print("cos:",leftAndRight)
    print("sin:",backAndForth)
    forward =1
    if backAndForth <0:
        forward=0
        backAndForth=-backAndForth

    right =1
    if leftAndRight <0 :
        right=0
        leftAndRight=-leftAndRight
    
    

    moveForwad = 255 * forward * backAndForth
    moveBackward = 255* (1-forward) * backAndForth

    moveRight =255 * right * leftAndRight
    moveLeft = 255 *(1-right) * leftAndRight
    
    print(f"F:{moveForwad:.1f} B:{moveBackward:.1f} R:{moveRight:.1f} L{moveLeft:.1f}")

   

if __name__ =="__main__":

    try:
        serialChannel = serial.Serial('/dev/ttyUSB0', 9600, timeout=2, 
                    parity=serial.PARITY_NONE, 
                    bytesize=serial.EIGHTBITS, 
                    stopbits=serial.STOPBITS_ONE)
        serialChannel.flush()
    except:
        print("USB not found")

    socketioSignal =SocketIOSignal(sioDomain=SIGNAL_SERVER,room="123",username="raspberry")

    robotRTC = RobotRTC(signal=socketioSignal,cameraVideo=PiCameraTrack(),serialCommunication=serialChannel)

    asyncio.run(robotRTC.start())

