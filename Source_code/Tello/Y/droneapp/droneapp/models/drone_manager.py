import logging
import contextlib
import os
import socket
import subprocess
import threading
import time

import cv2 as cv
import numpy as np

from droneapp.models.base import Singleton
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

        self.response = None
        self.stop_event = threading.Event()
        self._response_thread = threading.Thread(target=self.receive_response,
                                                 args=(self.stop_event,))
        self._response_thread.start()

        self.patrol_event = None
        self.is_patrol = False
        self._patrol_semaphore = threading.Semaphore(1)
        self._thread_patrol = None

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

        if not os.path.exists(SNAPSHOT_IMAGE_FOLDER):
            raise ErrorNoImageDir(f'{SNAPSHOT_IMAGE_FOLDER} does not exists')
        self.is_snapshot = False

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        self.send_command('command')
        self.send_command('streamon')
        self.set_speed(self.speed)

        # tracking a color
        green_lower = (30, 50, 50)
        green_upper = (80, 255, 255)
        # red_lower = (0, 50, 50)
        # red_upper = (20, 255, 255)
        # blue_lower = (110, 50, 50)
        # upper_blue = (130, 255, 255)

        self.xoff = 0
        self.yoff = 0
        self.tracker = Tracker(FRAME_X, FRAME_Y, green_lower, green_upper)
        self._is_enable_object_detect = False

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
                        self.up()
                    if status == 2:
                        self.clockwise(90)
                    if status == 3:
                        self.down()
                    if status == 4:
                        status = 0
                    time.sleep(5)
        else:
            logger.warning({'action': '_patrol', 'status': 'not_acquire'})

    def receive_video(self, stop_event, pipe_in, host_ip, video_port):
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
                    if percent_face > 0.30:
                        drone_x = -30
                    if percent_face < 0.02:
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

    def send_command_without_return(self, command: str):
        """Send command to Tello without expecting a response. Use this method when you want to send a command
        continuously
            - go x y z speed: Tello fly to x y z in speed (cm/s)
                x: 20-500
                y: 20-500
                z: 20-500
                speed: 10-100
            - curve x1 y1 z1 x2 y2 z2 speed: Tello fly a curve defined by the current and two given coordinates with
                speed (cm/s). If the arc radius is not within the range of 0.5-10 meters, it responses false.
                x/y/z can’t be between -20 – 20 at the same time .
                x1, x2: 20-500
                y1, y2: 20-500
                z1, z2: 20-500
                speed: 10-60
            - rc a b c d: Send RC control via four channels.
                a: left/right (-100~100)
                b: forward/backward (-100~100)
                c: up/down (-100~100)
                d: yaw (-100~100)
        """
        # Commands very consecutive makes the drone not respond to them. So wait at least self.TIME_BTW_COMMANDS seconds

        logger.info('Send command (no expect response): ' + command)
        self.socket.sendto(command.encode('utf-8'), self.drone_address)

    def route(self):
        status = 0
        self.takeoff()
        while not status == 10:
            status += 1
            drone_a = 0  # Left - right attributes -100 ~ 100
            drone_b = 0  # Backward - forward attributes -100 ~ 100
            drone_c = 0  # Down - up attributes attributes -100 ~ 100
            drone_d = 0  # Yaw attributes -100 ~ 100
            if status == 1:
                drone_b = 60
            if status == 2:
                drone_b = -20
            if status == 3:
                drone_d = 50
            if status == 4:
                drone_b = 20
            if status == 5:
                drone_b = -20
            if status == 6:
                drone_c = 40
            if status == 7:
                drone_c = -40
            if status == 8:
                drone_d = -50
            if status == 9:
                self.land()
                status = 10
            time.sleep(2)
            self.send_command(f'rc {drone_a} {drone_b} {drone_c} {drone_d}', blocking=False)

    def enable_object_detect(self):
        self._is_enable_object_detect = True

    def disable_object_detect(self):
        self._is_enable_object_detect = False

    def object_detection(self):
        for frame in self.video_binary_generator():
            if self._is_enable_object_detect:
                if self.is_patrol:
                    self.stop_patrol()
                if self._is_enable_face_detect:
                    self.disable_face_detect()
                """convert frame to cv2 image and show"""
                image = cv.cvtColor(np.array(
                    frame.to_image()), cv.COLOR_RGB2BGR)

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
