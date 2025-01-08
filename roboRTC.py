from abc import abstractmethod
import asyncio
import os
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
COMMAND_SERIAL_REQUEST="HEY!!!"

class PiCameraTrack(MediaStreamTrack):
    kind = "video"
    cam = None
    def __init__(self):
        super().__init__()
        try:
            self.cam =Picamera2()
            cam_config =self.cam.create_video_configuration( {"size":(640,480)},controls={"FrameRate": 30})
            self.cam.configure(cam_config)
            self.cam.start()
        except Exception as e:
            print(f"Camera exception {e}")

    def startToCapture(self):
        if None != self.cam:
            self.cam.start()
    def stopCapture(self):
        if None != self.cam:
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

class SerialCommunication():
    def __init__(self,nameToSearch,portPrefix):
        self.deviceName =nameToSearch
        self.deviceType=portPrefix

        self.maxErrorRetry=10
        self.serialPort:serial=None#/dev/ttyACM
        self.portname=None

        self.scanAndGetSerialPort()
        
        if None == self.portname:
            return 
        self.serialPort = serial.Serial(self.portname, 9600, write_timeout=2, 
                    parity=serial.PARITY_NONE, 
                    bytesize=serial.EIGHTBITS, 
                    stopbits=serial.STOPBITS_ONE)
        self.serialPort.flush()

    def scanAndGetSerialPort(self):
        for i in range(0,255):
            port =f"{self.deviceType}{i}"
            if os.path.exists(port):   
                with serial.Serial(port,9600, timeout=2) as ser:
                    try:
                        print(f"Port {port} is available and can be opened.")
                        
                        start_time = time.time()
                        while True:
                            ser.write(f"<{COMMAND_SERIAL_REQUEST}>" .encode('ascii'))
                            line = ser.readline().decode('ascii').strip()
                            print(f"---> {line}")
                            if line == self.deviceName:
                                print(f"got you {line}")
                                self.portname = port
                                break

                            if time.time() - start_time > 5:
                                print("Timeout: No response received.")
                                break
                    except serial.SerialException as e:
                        print(f" {port} not found ${e}")
                        pass
        

    def writeData(self,message):
        if None == self.serialPort:
            print("No serial port")
            return 
        print(f"send: {message}")

        try:
            if 0>= self.maxErrorRetry:
                self.reconnect()

            self.serialPort.write( f"<{message}>".encode('ascii'))
            self.serialPort.flush()
        except  serial.SerialException as e:
            self.maxErrorRetry =self.maxErrorRetry-1
            print(f"serial exception{e}")

    def reconnect(self):
        self.maxErrorRetry=10
        self.scanAndGetSerialPort()
        

class RobotRTC:
    signal : Signal =None
    def __init__(self,signal :Signal,cameraVideo:PiCameraTrack,serialCommunication:SerialCommunication,serialCommunicationHead:SerialCommunication):
   # def __init__(self,signal :Signal,cameraVideo:PiCameraTrack,serialCommunication:SerialCommunication,serialCommunicationHead:serial):
        self.signal =signal
        self.cameraVideo =cameraVideo
        self.audio = None
        self.serialCommunication =serialCommunication
        self.serialCommunicationHead =serialCommunicationHead
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
        async def on_message(message):     

            while True:

                x=0.0
                y=0.0
                r=1.0
                joystick ="LEFT"
                try:
                    data = json.loads(message)
                    joystick =data['joystick']
                except json.JSONDecodeError as e:
                    print("Invalid JSON:", e)

                self.deltaTime = int(time.time()*1000) - self.timeStamp
                if self.deltaTime >= self.cps:
                    # period between 2 command longer than the limit
                    # take timestamp at this time
                    # otherwise not take the timestamp
                    self.timeStamp=int(time.time()*1000)
                else:
                    try:
                        #{"joystick":"LEFT","x":0.0,"y":0.0}
                        if "LEFT" == joystick:
                            x=data['x']
                            y=data['y']
                            if x!=0.0 or y!=0.0:
                                # this is not the termial message #{"joystick":"LEFT","x":0.0,"y":0.0}#
                                break      
                            # else:
                            #     print("-------keep---------" )
                        elif "RIGHT" == joystick:
                            break    
                    except json.JSONDecodeError as e:
                        print("Invalid JSON:", e)
            
                if "LEFT" == joystick :
                    debugCommandMessge(message,True)
                    await sendDataToSerial(self.serialCommunication,message)
                    break
                if "RIGHT" == joystick :
                    await sendDataToSerial(self.serialCommunicationHead,message)
                    break

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

async def sendDataToSerialRaw(serialPort:serial,message):
    if None == serialPort:
        print("No serial port")
        return 
    print(f"send: {message}")
    try:

        serialPort.write( f"<{message}>".encode('ascii'))
        serialPort.flush()
    except  serial.SerialException as e:
        print(f"serial exception{e}")

async def sendDataToSerial(serialPort:SerialCommunication,message):
    if None == serialPort:
        print("No serial port")
        return 
    print(f"send: {message}")

    try:
        serialPort.serialPort.write( f"<{message}>".encode('ascii'))
        serialPort.serialPort.flush()
    except  serial.SerialException as e:
        print(f"serial exception{e}")



def debugCommandMessge(message, isShow :bool=False):
    
    exceptArc =math.pi/8

    if not isShow:
        return
    try:
        data = json.loads(message)
        x=data['x']
        y=data['y']
        r=data['r']       
        x_y_length=math.sqrt(x**2 + y**2)
        speedRatio =x_y_length/r
        rawArcRadian =-1*math.atan2(y,x)
 

        while True:
            
            if (rawArcRadian < exceptArc) and (rawArcRadian> -1 * exceptArc):
                rawArcRadian =0
                print("0pi-------------------")
                break
            
            if (rawArcRadian > math.pi/2 - exceptArc) and (rawArcRadian < math.pi/2 + exceptArc):
                rawArcRadian= math.pi/2
                print("pi/2------------------")
                break

            if (rawArcRadian > math.pi - exceptArc  and rawArcRadian < math.pi) or (rawArcRadian < -1* math.pi + exceptArc and rawArcRadian>-1* math.pi):
                rawArcRadian = math.pi
                print("pi ------------")
                break

            if( rawArcRadian < -1 * math.pi/2 + exceptArc) and (rawArcRadian > -1 * math.pi/2 - exceptArc):
                rawArcRadian = -1* math.pi/2
                print("-pi ----------------")
                break
            break
        
        
        backAndForth = math.sin(rawArcRadian)
        leftAndRight = math.cos(rawArcRadian)
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
        
        
        moveForwad = speedRatio * 255 * forward * backAndForth
        moveBackward = speedRatio* 255* (1-forward) * backAndForth

        moveRight =speedRatio * 255 * right * leftAndRight
        moveLeft =speedRatio*  255 *(1-right) * leftAndRight
        
        print(f"F:{moveForwad:.1f} B:{moveBackward:.1f} R:{moveRight:.1f} L{moveLeft:.1f}")

        
    except json.JSONDecodeError as e:
        print("Invalid JSON:", e)
    
serialChannelBody =None
serialChannelHead=None
if __name__ =="__main__":
    try:
        # serialChannelBody = serial.Serial('/dev/ttyUSB1', 9600, write_timeout=0, 
        #             parity=serial.PARITY_NONE, 
        #             bytesize=serial.EIGHTBITS, 
        #             stopbits=serial.STOPBITS_ONE)
        # serialChannelBody.flush()
        serialChannelBody =SerialCommunication("BODY","/dev/ttyUSB")

    except:
        print("USB Body nottttt found")
    try:
        serialChannelHead =SerialCommunication("HEAD","/dev/ttyUSB")

        # serialChannelHead = serial.Serial('/dev/ttyACM1', 9600, write_timeout=0, 
        #             parity=serial.PARITY_NONE, 
        #             bytesize=serial.EIGHTBITS, 
        #             stopbits=serial.STOPBITS_ONE)
        # serialChannelBody.flush()    
    except:
        print("USB Head Nottttttt found")

    socketioSignal =SocketIOSignal(sioDomain=SIGNAL_SERVER,room="123",username="raspberry")

    robotRTC = RobotRTC(signal=socketioSignal,cameraVideo=PiCameraTrack(),serialCommunication=serialChannelBody,serialCommunicationHead=serialChannelHead)

    asyncio.run(robotRTC.start())

