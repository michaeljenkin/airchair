import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class ArucoTarget(Node):
    _DICTS = {
        "4x4_100" : cv2.aruco.DICT_4X4_100,
        "4x4_1000" : cv2.aruco.DICT_4X4_1000,
        "4x4_250" : cv2.aruco.DICT_4X4_250,
        "4x4_50" : cv2.aruco.DICT_4X4_50,
        "5x5_100" : cv2.aruco.DICT_5X5_100,
        "5x5_1000" : cv2.aruco.DICT_5X5_1000,
        "5x5_250" : cv2.aruco.DICT_5X5_250,
        "5x5_50" : cv2.aruco.DICT_5X5_50,
        "6x6_100" : cv2.aruco.DICT_6X6_100,
        "6x6_1000" : cv2.aruco.DICT_6X6_1000,
        "6x6_250" : cv2.aruco.DICT_6X6_250,
        "6x6_50" : cv2.aruco.DICT_6X6_50,
        "7x7_100" : cv2.aruco.DICT_7X7_100,
        "7x7_1000" : cv2.aruco.DICT_7X7_1000,
        "7x7_250": cv2.aruco.DICT_7X7_250,
        "7x7_50": cv2.aruco.DICT_7X7_50,
        "apriltag_16h5" : cv2.aruco.DICT_APRILTAG_16H5,
        "apriltag_25h9" : cv2.aruco.DICT_APRILTAG_25H9,
        "apriltag_36h10" : cv2.aruco.DICT_APRILTAG_36H10,
        "apriltag_36h11" : cv2.aruco.DICT_APRILTAG_36H11,
        "aruco_original" : cv2.aruco.DICT_ARUCO_ORIGINAL
    }

    def __init__(self, tag_set="apriltag_36h10", target_width=0.20):
        super().__init__('aruco_target')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('image', "/chair_a/camera/image_raw")
        self.declare_parameter('info', "/chair_a/camera/camera_info")

        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._info_topic = self.get_parameter('info').get_parameter_value().string_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self.create_subscription(CameraInfo, self._info_topic, self._info_callback, 1)

        self._bridge = CvBridge()

        dict = ArucoTarget._DICTS.get(tag_set.lower(), None)
        self.get_logger().info(f'{self.get_name()} has dict {dict}')
        if dict is None:
            self.get_logger().error(f'ARUCO tag set {tag_set} not found')
        else:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict)
            self._aruco_param = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_param)
#            self._aruco_dict = cv2.aruco.Dictionary_get(dict)
#            self._aruco_param = cv2.aruco.DetectorParameters_create()
            self._target_width = target_width
            self._image = None
            self._cameraMatrix = None
            self.get_logger().info(f"using dictionary {tag_set}")

    def _info_callback(self, msg):
        if msg.distortion_model != "plumb_bob":
            self.get_logger().error(f"We can only deal with plumb_bob distortion {msg.distortion_model}")
        self._distortion = np.reshape(msg.d, (1,5))
        self._cameraMatrix = np.reshape(msg.k, (3,3))

    def _estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
        marker_points = np.array([ [-marker_size/2, marker_size/2, 0],
                                   [marker_size/2, marker_size/2, 0],
                                   [marker_size/2, -marker_size/2, 0],
                                   [-marker_size/2, -marker_size/2, 0]], dtype=np.float32)
        trash =[]
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def _image_callback(self, msg):
        self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 

        grey = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self._aruco_detector.detectMarkers(grey)
#        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(grey, self._aruco_dict)
        frame = cv2.aruco.drawDetectedMarkers(self._image, corners, ids)
        if ids is None:
            self.get_logger().info(f"No targets found!")
            return
        if self._cameraMatrix is None:
            self.get_logger().info(f"We have not yet received a camera_info message")
            return

        rvec, tvec, _objPoints = self._estimatePoseSingleMarkers(corners, self._target_width, self._cameraMatrix, self._distortion)
        self.get_logger().info(f"We got back {rvec} {tvec}")
#        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self._target_width, self._cameraMatrix, self._distortion)
        result = self._image.copy()
        for r,t in zip(rvec,tvec):
            self.get_logger().info(f"Found a target at {t} rotation {r}")
            result = cv2.drawFrameAxes(result, self._cameraMatrix, self._distortion, r, t, self._target_width)
#            result = cv2.aruco.drawAxis(result, self._cameraMatrix, self._distortion, r, t, self._target_width)
        cv2.imshow('window', result)
#        cv2.imshow('window', frame)
        cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

