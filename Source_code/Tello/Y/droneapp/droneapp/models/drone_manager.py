import logging
import contextlib
import os
import socket
import subprocess
import threading
import time
import datetime

import cv2 as cv
import numpy as np
import tellopy
import av

from droneapp.models.base import Singleton
from droneapp.djitellopy.tello import Tello
from droneapp.controllers.tracker import Tracker

logger = logging.getLogger(__name__)

DEFAULT_DISTANCE = 0.30
DEFAULT_SPEED = 10
DEFAULT_DEGREE = 10

FRAME_X = int(960 / 3)
FRAME_Y = int(720 / 3)
FRAME_AREA = FRAME_X * FRAME_Y

FRAME_SIZE = FRAME_AREA * 3
FRAME_CENTER_X = FRAME_X / 2
FRAME_CENTER_Y = FRAME_Y / 2

CMD_FFMPEG = (f'ffmpeg -hwaccel auto -hwaccel_device opencl -i pipe:0 '
              f'-pix_fmt bgr24 -s {FRAME_X}x{FRAME_Y} -f rawvideo pipe:1')

FACE_DETECT_XML_FILE = './droneapp/models/haarcascade_frontalface_default.xml'
OBJECT_DETECT_FILE = './droneapp/models/object_default.png'

SNAPSHOT_IMAGE_FOLDER = './droneapp/static/img/snapshots/'


class ErrorNoFaceDetectXMLFile(Exception):
    """Error no face detect xml file"""


class ErrorNoImageDir(Exception):
    """Error no image dir"""


class DroneManager(metaclass=Singleton):
    def __init__(self, host_ip='192.168.10.2', host_port=8889,
                 drone_ip='192.168.10.1', drone_port=8889,
                 is_imperial=False, speed=DEFAULT_SPEED):
        self.host_ip = host_ip
        self.host_port = host_port
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.drone_address = (drone_ip, drone_port)
        self.is_imperial = is_imperial
        self.speed = speed
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host_ip, self.host_port))

        self.date_fmt = '%Y-%m-%d_%H%M%S'
        self.drone = tellopy.Tello()
        self.init_drone()
        self.tello = Tello()
        self.tracking = False

        # container for processing the packets into frames
        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]
        self.out_file = None
        self.out_stream = None
        self.out_name = None
        self.start_time = time.time()

        # tracking a color
        green_lower = (30, 50, 50)
        green_upper = (80, 255, 255)
        # red_lower = (0, 50, 50)
        # red_upper = (20, 255, 255)
        # blue_lower = (110, 50, 50)
        # upper_blue = (130, 255, 255)
        self.xoff = 0
        self.yoff = 0
        self.track_cmd = ""
        self.tracker = Tracker(self.vid_stream.height,
                               self.vid_stream.width,
                               green_lower, green_upper)
        self.response = None
        self.stop_event = threading.Event()
        self._response_thread = threading.Thread(target=self.receive_response, args=(self.stop_event,))
        self._response_thread.start()

        self.patrol_event = None
        self.is_patrol = False
        self._patrol_semaphore = threading.Semaphore(1)
        self._thread_patrol = None

        self.routing_event = None
        self.is_routing = False
        self._routing_semaphore = threading.Semaphore(1)
        self._thread_routing = None

        self.proc = subprocess.Popen(CMD_FFMPEG.split(' '),
                                     stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE)
        self.proc_stdin = self.proc.stdin
        self.proc_stdout = self.proc.stdout

        self.video_port = 11111

        self._receive_video_thread = threading.Thread(
            target=self.receive_video,
            args=(self.stop_event, self.proc_stdin,
                  self.host_ip, self.video_port,))
        self._receive_video_thread.start()

        if not os.path.exists(FACE_DETECT_XML_FILE):
            raise ErrorNoFaceDetectXMLFile(f'No {FACE_DETECT_XML_FILE}')
        self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        self._is_enable_face_detect = False
        self._is_enable_object_detect = False

        if not os.path.exists(SNAPSHOT_IMAGE_FOLDER):
            raise ErrorNoImageDir(f'{SNAPSHOT_IMAGE_FOLDER} does not exists')
        self.is_snapshot = False

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        self.send_command('command')
        self.send_command('streamon')
        self.set_speed(self.speed)

    def init_drone(self):
        """Connect, unable streaming and subscribe to events"""
        # self.drone.log.set_level(2)
        self.drone.connect()
        self.drone.start_video()
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,
                             self.flight_data_handler)
        self.drone.subscribe(self.drone.EVENT_FILE_RECEIVED,
                             self.handle_flight_received)

    def receive_response(self, stop_event):
        while not stop_event.is_set():
            try:
                self.response, ip = self.socket.recvfrom(3000)
                logger.info({'action': 'receive_response',
                             'response': self.response})
            except socket.error as ex:
                logger.error({'action': 'receive_response',
                              'ex': ex})
                break

    def __dell__(self):
        self.stop()

    def stop(self):
        self.stop_event.set()
        retry = 0
        while self._response_thread.isAlive():
            time.sleep(0.3)
            if retry > 30:
                break
            retry += 1
        self.socket.close()
        os.kill(self.proc.pid, 9)
        # Windows
        # import signal
        # os.kill(self.proc.pid, signal.CTRL_C_EVENT)

    def send_command(self, command, blocking=True):
        self._command_thread = threading.Thread(
            target=self._send_command,
            args=(command, blocking,))
        self._command_thread.start()

    def _send_command(self, command, blocking=True):
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                logger.info({'action': 'send_command', 'command': command})
                self.socket.sendto(command.encode('utf-8'), self.drone_address)

                retry = 0
                while self.response is None:
                    time.sleep(0.3)
                    if retry > 3:
                        break
                    retry += 1

                if self.response is None:
                    response = None
                else:
                    response = self.response.decode('utf-8')
                self.response = None
                return response

        else:
            logger.warning({'action': 'send_command', 'command': command, 'status': 'not_acquire'})

    def takeoff(self):
        return self.send_command('takeoff')

    def land(self):
        return self.send_command('land')

    def move(self, direction, distance):
        distance = float(distance)
        if self.is_imperial:
            distance = int(round(distance * 30.48))
        else:
            distance = int(round(distance * 100))
        return self.send_command(f'{direction} {distance}')

    def up(self, distance=DEFAULT_DISTANCE):
        return self.move('up', distance)

    def down(self, distance=DEFAULT_DISTANCE):
        return self.move('down', distance)

    def left(self, distance=DEFAULT_DISTANCE):
        return self.move('left', distance)

    def right(self, distance=DEFAULT_DISTANCE):
        return self.move('right', distance)

    def forward(self, distance=DEFAULT_DISTANCE):
        return self.move('forward', distance)

    def back(self, distance=DEFAULT_DISTANCE):
        return self.move('back', distance)

    def set_speed(self, speed):
        return self.send_command(f'speed {speed}')

    def clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'cw {degree}')

    def counter_clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'ccw {degree}')

    def flip_front(self):
        return self.send_command('flip f')

    def flip_back(self):
        return self.send_command('flip b')

    def flip_left(self):
        return self.send_command('flip l')

    def flip_right(self):
        return self.send_command('flip r')

    def patrol(self):
        if not self.is_patrol:
            self.patrol_event = threading.Event()
            self._thread_patrol = threading.Thread(
                target=self._patrol,
                args=(self._patrol_semaphore, self.patrol_event,))
            self._thread_patrol.start()
            self.is_patrol = True

    def stop_patrol(self):
        if self.is_patrol:
            self.patrol_event.set()
            retry = 0
            while self._thread_patrol.isAlive():
                time.sleep(0.3)
                if retry > 300:
                    break
                retry += 1
            self.is_patrol = False

    def _patrol(self, semaphore, stop_event):
        is_acquire = semaphore.acquire(blocking=False)
        if is_acquire:
            logger.info({'action': '_patrol', 'status': 'acquire'})
            with contextlib.ExitStack() as stack:
                stack.callback(semaphore.release)
                status = 0
                while not stop_event.is_set():
                    status += 1
                    if status == 1:
                        self.forward(0.5)
                    if status == 2:
                        self.clockwise(90)
                    if status == 3:
                        status = 0
                        # break;
                    #     test break if status == 0 and drone not completely land? 
                    time.sleep(2)
        else:
            logger.warning({'action': '_patrol', 'status': 'not_acquire'})

    @staticmethod
    def receive_video(stop_event, pipe_in, host_ip, video_port):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock_video:
            sock_video.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock_video.settimeout(.5)
            sock_video.bind((host_ip, video_port))
            data = bytearray(2048)
            while not stop_event.is_set():
                try:
                    size, addr = sock_video.recvfrom_into(data)
                    # logger.info({'action': 'receive_video', 'data': data})
                except socket.timeout as ex:
                    logger.warning({'action': 'receive_video', 'ex': ex})
                    time.sleep(0.5)
                    continue
                except socket.error as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    break

                try:
                    pipe_in.write(data[:size])
                    pipe_in.flush()
                except Exception as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    break

    def video_binary_generator(self):
        while True:
            try:
                frame = self.proc_stdout.read(FRAME_SIZE)
            except Exception as ex:
                logger.error({'action': 'video_binary_generator', 'ex': ex})
                continue

            if not frame:
                continue

            frame = np.fromstring(frame, np.uint8).reshape(FRAME_Y, FRAME_X, 3)
            yield frame

    def enable_face_detect(self):
        self._is_enable_face_detect = True

    def disable_face_detect(self):
        self._is_enable_face_detect = False

    def video_jpeg_generator(self):
        for frame in self.video_binary_generator():
            if self._is_enable_face_detect:
                if self.is_patrol:
                    self.stop_patrol()
                if self._is_enable_object_detect:
                    self.disable_object_detect()

                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    face_center_x = x + (w / 2)
                    face_center_y = y + (h / 2)
                    diff_x = FRAME_CENTER_X - face_center_x
                    diff_y = FRAME_CENTER_Y - face_center_y
                    face_area = w * h
                    percent_face = face_area / FRAME_AREA

                    drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
                    if diff_x < -30:
                        drone_y = -30
                    if diff_x > 30:
                        drone_y = 30
                    if diff_y < -15:
                        drone_z = -30
                    if diff_y > 15:
                        drone_z = 30
                    if percent_face > 0.40:
                        drone_x = -30
                    if percent_face < 0.1:
                        drone_x = 30
                    self.send_command(f'go {drone_x} {drone_y} {drone_z} {speed}',
                                      blocking=False)
                    break

            _, jpeg = cv.imencode('.jpg', frame)
            jpeg_binary = jpeg.tobytes()

            if self.is_snapshot:
                backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
                snapshot_file = 'snapshot.jpg'
                for filename in (backup_file, snapshot_file):
                    file_path = os.path.join(
                        SNAPSHOT_IMAGE_FOLDER, filename)
                    with open(file_path, 'wb') as f:
                        f.write(jpeg_binary)
                self.is_snapshot = False

            yield jpeg_binary

    def snapshot(self):
        self.is_snapshot = True
        retry = 0
        while retry < 3:
            if not self.is_snapshot:
                return True
            time.sleep(0.1)
            retry += 1
        return False

    def route(self, stop_event):
        drone_x = 0
        drone_y = 0
        drone_z = 0
        speed = self.speed
        self.takeoff()
        drone_y = 0.3
        time.sleep(2)
        self.tello.send_command_without_return(f'go {drone_x} {drone_y} {drone_z} {speed}')
        drone_y = -0.3
        time.sleep(2)
        self.tello.send_command_without_return(f'go {drone_x} {drone_y} {drone_z} {speed}')
        drone_z = 0.3
        time.sleep(2)
        self.tello.send_command_without_return(f'go {drone_x} {drone_y} {drone_z} {speed}')
        drone_z = -0.3
        time.sleep(2)
        self.tello.send_command_without_return(f'go {drone_x} {drone_y} {drone_z} {speed}')
        time.sleep(2)
        self.land()

    def enable_object_detect(self):
        self._is_enable_object_detect = True

    def disable_object_detect(self):
        self._is_enable_object_detect = False

    def object_detection(self):
        for packet in self.container.demux((self.vid_stream,)):
            for frame in packet.decode():
                if self._is_enable_face_detect:
                    if self.is_patrol:
                        self.stop_patrol()
                    if self._is_enable_face_detect:
                        self.disable_face_detect()
                    """convert frame to cv2 image and show"""
                    image = cv.cvtColor(np.array(
                        frame.to_image()), cv.COLOR_RGB2BGR)
                    image = self.write_hud(image)

                    xoff, yoff, object_area = self.tracker.track(image)
                    object_percentage = object_area / FRAME_AREA

                    drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
                    distance = 40
                    if xoff < -distance:
                        drone_y = -distance
                    if xoff > distance:
                        drone_y = distance
                    if yoff < -distance / 2:
                        drone_z = -distance
                    if yoff > distance / 2:
                        drone_z = distance
                    if object_percentage > 0.40:
                        drone_x = -distance
                    if object_percentage < 0.10:
                        drone_x = distance
                    self.send_command(f'go {drone_x} {drone_y} {drone_z} {speed}',
                                      blocking=False)
                    break

            _, image = cv.imencode('.jpg', frame)
            image_binary = image.tobytes()

            yield image_binary

    def write_hud(self, frame):
        """Draw drone info, tracking and record on frame"""
        stats = self.prev_flight_data.split('|')
        stats.append("Tracking:" + str(self.tracking))
        if self.drone.zoom:
            stats.append("VID")
        else:
            stats.append("PIC")

        for idx, stat in enumerate(stats):
            text = stat.lstrip()
            cv.putText(frame, text, (0, 30 + (idx * 30)),
                        cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), lineType=30)
        return frame

    def palm_land(self, speed):
        """Tell drone to land"""
        if speed == 0:
            return
        self.drone.palm_land()

    def toggle_tracking(self, speed):
        """ Handle tracking keypress"""
        if speed == 0:  # handle key up event
            return
        self.tracking = not self.tracking
        print("tracking:", self.tracking)
        return

    def toggle_zoom(self, speed):
        """
        In "video" mode the self.drone sends 1280x720 frames.
        In "photo" mode it sends 2592x1936 (952x720) frames.
        The video will always be centered in the window.
        In photo mode, if we keep the window at 1280x720 that gives us ~160px on
        each side for status information, which is ample.
        Video mode is harder because then we need to abandon the 16:9 display size
        if we want to put the HUD next to the video.
        """
        if speed == 0:
            return
        self.drone.set_video_mode(not self.drone.zoom)

    def flight_data_handler(self, event, sender, data):
        """Listener to flight data from the drone."""
        text = str(data)
        if self.prev_flight_data != text:
            self.prev_flight_data = text

    def handle_flight_received(self, event, sender, data):
        """Create a file in ~/Pictures/ to receive image from the drone"""
        path = '%s/Pictures/tello-%s.jpeg' % (
            os.getenv('HOME'),
            datetime.datetime.now().strftime(self.date_fmt))
        with open(path, 'wb') as out_file:
            out_file.write(data)
        print('Saved photo to %s' % path)