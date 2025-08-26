import rclpy
from rclpy.node import Node
from pynput import keyboard
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

ID_TO_FIND = 72
MARKER_SIZE = 0.2 

NP_CAMERA_MATRIX = np.array([
    [467.74270306499267, 0.0, 320.5],
    [0.0, 467.74270306499267, 240.5],
    [0.0, 0.0, 1.0]
], dtype=np.float32)
NP_DIST_COEFF = np.zeros((5,), dtype=np.float32)

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
ARUCO_PARAMS = aruco.DetectorParameters()

HORIZONTAL_RES = 640
VERTICAL_RES = 480
HORIZONTAL_FOV = 68.75 * (math.pi / 180)
VERTICAL_FOV = 54.32 * (math.pi / 180)

TAKEOFF_HEIGHT = 3.0
VELOCITY = 0.5
CENTER_TOLERANCE = 0.01 
LAND_THRESHOLD = 0.2 

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.bridge = CvBridge()
        self.vehicle = None
        self.marker_found = False
        self.is_flying = False
        self.manual_control = True
        self.target_altitude = 0.0
        self.last_process_time = time.time()
        self.process_interval = 0.05
        self.marker_corners = None   
        self.rvec = None              
        self.tvec = None      
        self.manual_keys = set()      

        self.publisher_processed = self.create_publisher(Image, '/processed/image', 10)

        gst_str = (
            "udpsrc address=0.0.0.0 port=5600 ! "
            "application/x-rtp,encoding-name=H264,payload=96 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "appsink drop=1 sync=false"
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
            raise RuntimeError("Could not open GStreamer pipeline!")
        self.get_logger().info("Camera is on.")

        self.connect_vehicle()
        if self.vehicle:
            self.get_logger().info("Drone connected.")

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        self.timer = self.create_timer(0.033, self.timer_callback)

    def connect_vehicle(self):
        self.get_logger().info('Connecting drone...')
        try:
            self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True, timeout=60)
            self.vehicle.parameters['PLND_ENABLED'] = 1
            self.vehicle.parameters['PLND_TYPE'] = 1
            self.vehicle.parameters['PLND_EST_TYPE'] = 0
            self.vehicle.parameters['LAND_SPEED'] = 30
            self.get_logger().info('Drone connected and PLND parameters set.')
        except Exception as e:
            self.get_logger().error(f"Drone connection failed: {e}")
            self.vehicle = None

    # Manual Flight
    def arm_and_takeoff(self, altitude):
        self.get_logger().info("Drone is being armed")
        while not self.vehicle.is_armable:
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.get_logger().info("It is expected to be arm...")
            time.sleep(1)

        self.get_logger().info(f"Engines started, Takeoff begins at {altitude} m")
        self.vehicle.simple_takeoff(altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if abs(alt - altitude) <= 0.5:
                self.is_flying = True
                self.target_altitude = alt
                self.get_logger().info("Takeoff completed!")
                break
            time.sleep(0.5)

    def increase_altitude(self, delta):
        self.target_altitude += delta
        self.send_ned_velocity(0, 0, -0.5, delta/0.5)

    def send_ned_velocity(self, vx, vy, vz, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        for _ in range(int(duration*10)):
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)
        
    def stop_movement(self):
        self.send_ned_velocity(0, 0, 0, 0.1)

    def land_drone(self):
        self.vehicle.mode = VehicleMode("LAND")
        self.is_flying = False
        self.get_logger().info("Drone lands!")

    def on_press(self, key):
        try:
            if key.char in ['w','a','s','d','e']:
                self.manual_keys.add(key.char)
            if key.char == 'f' and not self.vehicle.armed:
                threading.Thread(target=self.arm_and_takeoff, args=(1.0,)).start()
            elif key.char == 'q' and self.is_flying:
                self.land_drone()

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in self.manual_keys:
                self.manual_keys.remove(key.char)
            if not self.manual_keys: 
                self.stop_movement()
        except AttributeError:
            pass

    #  Marker Tracking
    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_process_time < self.process_interval:
            return
        self.last_process_time = current_time

        if self.is_flying and self.manual_keys:
            if 'w' in self.manual_keys:
                self.send_ned_velocity(VELOCITY, 0, 0, 0.1)
            if 's' in self.manual_keys:
                self.send_ned_velocity(-VELOCITY, 0, 0, 0.1)
            if 'd' in self.manual_keys:
                self.send_ned_velocity(0, VELOCITY, 0, 0.1)
            if 'a' in self.manual_keys:
                self.send_ned_velocity(0, -VELOCITY, 0, 0.1)
            if 'e' in self.manual_keys:
                self.send_ned_velocity(0, 0, -VELOCITY, 0.1)

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Frame could not be received!")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == ID_TO_FIND:
                    self.get_logger().info(f'Target marker found! ID: {marker_id}')
                    self.marker_found = True
                    marker_corners = corners[i]
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, NP_CAMERA_MATRIX, NP_DIST_COEFF)
                    rvec, tvec = rvec[0,0,:], tvec[0,0,:]

                    x_avg = np.mean(marker_corners[0][:,0])
                    y_avg = np.mean(marker_corners[0][:,1])

                    x_ang = (x_avg - HORIZONTAL_RES/2) * HORIZONTAL_FOV / HORIZONTAL_RES
                    y_ang = (y_avg - VERTICAL_RES/2) * VERTICAL_FOV / VERTICAL_RES

                    vx = x_ang * VELOCITY   
                    vy = y_ang * VELOCITY  

                    if tvec[2] > 1.0:
                        vz = 0.3
                    elif tvec[2] > 0.3:
                        vz = 0.1
                    else:
                        vz = 0

                    if self.vehicle.mode.name != 'LAND' and tvec[2] > LAND_THRESHOLD:
                        self.vehicle.mode = VehicleMode('GUIDED')

                    if tvec[2] <= LAND_THRESHOLD or (abs(x_ang) < CENTER_TOLERANCE and abs(y_ang) < CENTER_TOLERANCE and tvec[2] < 1.0):
                        if self.vehicle.mode.name != 'LAND':
                            self.vehicle.mode = VehicleMode('LAND')

                    self.send_ned_velocity(vx, vy, vz, duration=0.1)

                    aruco.drawDetectedMarkers(frame, [marker_corners])
                    cv2.drawFrameAxes(frame, NP_CAMERA_MATRIX, NP_DIST_COEFF, rvec, tvec, 0.1)
                    cv2.putText(frame, f'X={tvec[0]:.2f} Y={tvec[1]:.2f} Z={tvec[2]:.2f}', (10,50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0),2)

        msg_processed = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_processed.publish(msg_processed)
        cv2.imshow("DRONE CAM", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("The program is terminated with the 'q' key.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.vehicle:
            node.vehicle.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
