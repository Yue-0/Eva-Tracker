from time import sleep

import cv2
import rospy
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge

from tracker.msg import Camera as Message

__author__ = "YueLin"


class RealSense:
    """RealSense Camera"""
    def __init__(self, fps: int = 30, width: int = 640, height: int = 480):
        self.fps = fps
        self.resolution = (width, height)

        # Setting up the camera
        self.camera, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Open the camera
        cfg = self.camera.start(cfg)
        sleep(1)

        # Depth map aligned to color map
        self.align = rs.align(rs.stream.color)

        # Get intrinsic
        intrinsic = cfg.get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
        self.fx = intrinsic.fx
        self.fy = intrinsic.fy
        self.cx = intrinsic.ppx
        self.cy = intrinsic.ppy

        # Get the scale of depth
        self.scale = cfg.get_device().first_depth_sensor().get_depth_scale()
    
    def read(self) -> tuple:
        """Read the current RGB image and depth image"""
        frames = self.align.process(self.camera.wait_for_frames())
        return tuple(map(lambda frame: cv2.rotate(  # Camera reverse dressing
            np.asarray(frame.get_data()), cv2.ROTATE_180
        ), (frames.get_depth_frame(), frames.get_color_frame())))
    
    def close(self) -> None:
        """Release camera"""
        self.camera.stop()


class CameraNode:
    """ROS camera node"""
    def __init__(self, node: str):
        
        # Initialize ROS node
        rospy.init_node(node)

        # Initialize RealSense camera
        self.camera = RealSense(
            rospy.get_param("~fps", 30),
            rospy.get_param("~width", 640), 
            rospy.get_param("~height", 480)
        )
        self.sleep = rospy.Rate(self.camera.fps).sleep

        # Initialize message
        self.message = Message()
        self.message.fx = self.camera.fx
        self.message.fy = self.camera.fy
        self.message.cx = self.camera.cx
        self.message.cy = self.camera.cy
        self.message.scale = self.camera.scale
        self.publisher = rospy.Publisher(
            "/tracker/camera", Message, queue_size=1
        )
        self.msg = CvBridge().cv2_to_imgmsg
    
    def run(self) -> None:
        """Publish message"""
        while not rospy.is_shutdown():
            depth, color = self.camera.read()
            self.message.color = self.msg(color, "bgr8")
            self.message.depth = self.msg(cv2.medianBlur(depth, 3), "mono16")
            self.publisher.publish(self.message)
            self.sleep()
        self.camera.close()


if __name__ == "__main__":
    CameraNode("camera").run()
