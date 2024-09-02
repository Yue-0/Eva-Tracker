import os
import time

import tf
import cv2
import rospy
import numpy as np
import onnxruntime as ort
import pyrealsense2 as rs
from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs import msg, point_cloud2
from nav_msgs.msg import Odometry as Odom

__author__ = "YueLin"


class RealSense:
    """RealSense Camera"""
    def __init__(self, fps: int = 30, width: int = 640, height: int = 480):
        # Setting up the camera
        self.camera, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Open the camera
        cfg = self.camera.start(cfg), time.sleep(1)

        # Depth map aligned to color map
        self.align = rs.align(rs.stream.color)

        # Get the camera's intrinsic matrix
        intrinsics = cfg[0].get_stream(
            rs.stream.color
        ).as_video_stream_profile().get_intrinsics()
        self.intrinsic = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy], [0, 0, 1]
        ])
    
    def read(self) -> map:
        """Read the current RGB image and depth image."""
        frames = self.align.process(self.camera.wait_for_frames())
        return map(
            lambda frame: np.asarray(frame.get_data()),
            (frames.get_color_frame(), frames.get_depth_frame())
        )
    
    def close(self) -> None:
        """Release camera."""
        self.camera.stop()


class Segmentor:
    """YOLOv8 Segmentation without ultralytics library"""
    def __init__(self, model: str, device: str = ""):
        path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "model")
        if not device:
            device = "CUDA" if ort.get_device() == "GPU" else "CPU"
        self.model = ort.InferenceSession(
            os.path.join(path, model), 
            providers=["{}ExecutionProvider".format(device.upper())]
        )
        self.masks = 0x20
        i = self.model.get_inputs()[0]
        self.input, self.shape = i.name, i.shape[-2:]

        # For visualization
        self.colors = [[int(h[i + 1:3 + i], 16) for i in (4, 2, 0)] for h in (
            "042AFF", "0BDBEB", "F3F3F3", "00DFB7", "111F68",
            "FF6FDD", "FF444F", "CCED00", "00F344", "BD00FF",
            "00B4FF", "DD00BA", "00FFFF", "26C000", "01FFB3",
            "7D24FF", "7B0068", "FF1B6C", "FC6D2F", "A2FF0B"
        )]
        self.classes = open(
            os.path.join(path, "classes.txt")
        ).read().rstrip().split('\n')
    
    def __call__(self,
                 image: np.ndarray,
                 threshold: float = 0.4,
                 nms: float = 0.45,
                 one: bool = False,
                 detections: tuple = None) -> tuple:
        """Perform an instance segmentation"""
        shape = image.shape
        image, ratio, pad = self.__preprocess(image)
        return self.__output(
            self.model.run(None, {self.input: image}),
            shape, pad, ratio, threshold, nms, one, detections
        )
    
    def visualize(self, 
                  image: np.ndarray, 
                  boxes: np.ndarray, 
                  segments: list) -> np.ndarray:
        """Visualize results."""
        mask = image.copy()

        # Draw rectangles and polygons
        for (*box, score, c), segment in zip(boxes, segments):
            color = self.colors[int(c) % len(self.colors)]

            # Draw contour and fill mask
            cv2.polylines(image, np.int32([segment]), True, (0xFF,) * 3, 2)
            cv2.fillPoly(mask, np.int32([segment]), color)

            # Draw rectangle
            cv2.rectangle(
                image, (int(box[0]), int(box[1])),
                (int(box[2]), int(box[3])), color, 1, cv2.LINE_AA,
            )
            cv2.putText(
                image, f"{self.classes[int(c)]}: {score:.2f}",
                (int(box[0]), int(box[1] - 1e1)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                color, 2, cv2.LINE_AA,
            )

        # Mix image and mask
        return cv2.addWeighted(mask, 0.3, image, 0.7, 0)

    def __preprocess(self, image: np.ndarray) -> tuple:
        """Preprocesses the input image."""
        r = min(self.shape[axis] / image.shape[axis] for axis in range(2)) 
        size = int(round(image.shape[1] * r)), int(round(image.shape[0] * r))
        pad = [(self.shape[1] - size[0]) >> 1, (self.shape[0] - size[1]) >> 1]

        # Resize and padding
        if size != image.shape[1::-1]:
            image = cv2.resize(image, size)
        image = cv2.copyMakeBorder(
            image, pad[1], pad[1], pad[0], pad[0], 
            cv2.BORDER_CONSTANT, value=(0x72,) * 3
        )
        if self.shape != image.shape[:2]:
            image = cv2.resize(image, self.shape[::-1])
        
        # Transforms
        return np.ascontiguousarray(
            np.einsum("HWC->CHW", image)[::-1], dtype=np.single
        )[np.newaxis] / 255., r, pad
    
    def __output(self, predicts: tuple, shape: tuple, pad: tuple, ratio: float, 
                 threshold: float, nms: float, one: bool = False, 
                 detections: tuple = None) -> tuple:
        """Post-process the predictions."""
        out = np.einsum("BCN->BNC", predicts[0])

        # Predictions filtering by threshold
        out = out[np.amax(out[..., 4:-self.masks], axis=-1) > threshold]

        # Merge (boxes, scores, classes, masks) into one matrix
        out = np.c_[
            out[..., :4], 
            np.amax(out[..., 4:-self.masks], axis=-1), 
            np.argmax(out[..., 4:-self.masks], axis=-1), out[..., -self.masks:]
        ]

        # Filter by specified category
        if out.shape[0] > 0 and detections:
            keep = np.array([False] * out.shape[0])
            for detection in detections:
                keep = np.logical_or(keep, out[:, 5] == detection)
            out = out[keep]
        
        # Filter the object with the highest confidence
        if out.shape[0] > 0 and one:
            out = out[np.argmax(out[:, 4])][np.newaxis]

        # NMS filtering
        if out.shape[0] > 0 and nms:
            out = out[cv2.dnn.NMSBoxes(out[:, :4], out[:, 4], threshold, nms)]

        # Decode
        if out.shape[0] > 0:
            # x0y0wh -> x1y1x2y2
            out[..., [0, 1]] -= out[..., [2, 3]] / 2
            out[..., [2, 3]] += out[..., [0, 1]]

            # Rescale boxes to the shape of original image
            out[..., :4] -= [pad[0], pad[1], pad[0], pad[1]]
            out[..., :4] /= ratio

            # Boundary clip
            out[..., [0, 2]] = out[:, [0, 2]].clip(0, shape[1])
            out[..., [1, 3]] = out[:, [1, 3]].clip(0, shape[0])

            # Get masks
            masks = self.__mask(predicts[1][0], out[:, 6:], out[:, :4], shape)

            # Masks -> Contours
            contours = []
            for mask in masks.astype(np.uint8):
                contour = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                )[0]
                contours.append(np.float32(np.array(
                    contour[np.argmax([len(c) for c in contour])]
                ).reshape(-1, 2) if contour else np.zeros((0, 2))))

            return out[..., :6], contours, masks
        return np.array([]), [], np.array([])
    
    def __mask(self, 
               outs: np.ndarray, 
               masks: np.ndarray, 
               boxes: np.ndarray, 
               shape: tuple) -> np.ndarray:
        """Takes the output of the mask head, and applies the mask to boxes."""
        masks = np.ascontiguousarray(np.matmul(
            masks, outs.reshape((outs.shape[0], -1))
        ).reshape((-1, *outs.shape[1:])).transpose(1, 2, 0))
        
        # Calculate pad of mask
        h, w = masks.shape[:2]
        ratio = min(h / shape[0], w / shape[1])
        pad = (w - shape[1] * ratio) / 2, (h - shape[0] * ratio) / 2

        # Resize mask to original input image shape
        masks = cv2.resize(masks[
            int(round(pad[1])):int(round(h - pad[1])), 
            int(round(pad[0])):int(round(w - pad[0]))
        ], shape[1::-1], interpolation=cv2.INTER_LINEAR)
        # ], shape[1::-1], interpolation=cv2.CUBIC_LINEAR)  # Better but slower
        if len(masks.shape) == 2:
            masks = masks[:, :, np.newaxis]
        masks = np.einsum("HWN->NHW", masks)
        
        # Crop mask
        x1, y1, x2, y2 = np.split(boxes[:, :, np.newaxis], 4, 1)
        r = np.arange(masks.shape[2], dtype=x1.dtype)[np.newaxis, np.newaxis, :]
        c = np.arange(masks.shape[1], dtype=y1.dtype)[np.newaxis, :, np.newaxis]
        
        return masks * ((r >= x1) * (r < x2) * (c >= y1) * (c < y2)) > 0.5


class Camera:
    """Camera ROS Node"""
    def __init__(self, node: str, model: str):
        rospy.init_node(node)
        self.sense = RealSense()
        self.model = Segmentor(model)
        self.frame = tf.TransformListener()
        self.np2im = CvBridge().cv2_to_imgmsg
        self.d_max = rospy.get_param("~distance_max", 3.0)
        self.world = rospy.get_param("~frame/world", "world")
        self.robot = rospy.get_param("~frame/robot", "tracker")
        self.sleep = rospy.Rate(rospy.get_param("~fps", 10)).sleep
        self.image = rospy.Publisher("/image", msg.Image, queue_size=1)
        self.track = rospy.Publisher("/target/odom", Odom, queue_size=1)
        self.depth = rospy.Publisher("/depth", msg.PointCloud2, queue_size=1)
    
    def run(self):
        intrinsics = np.linalg.inv(self.sense.intrinsic)
        self.frame.waitForTransform(
            self.robot, self.world, rospy.Time(), rospy.Duration(10)
        )

        # Initialize messages
        odom = Odom()
        header = Header()
        odom.pose.pose.orientation.w = 1
        odom.header.frame_id = self.world
        header.frame_id = rospy.get_param("~frame/lidar", "lidar")
        odom.child_frame_id = rospy.get_param("~frame/target", "target")

        # Main loop
        while not rospy.is_shutdown():
            # Get the robot's position
            (x, y, z), yaw = self.frame.lookupTransform(
                self.world, self.robot, rospy.Time()
            )
            yaw = tf.transformations.euler_from_quaternion(yaw)[-1]

            # Get RGB-D image
            color, depth = self.sense.read()
            depth = depth * 1e-3
            # depth = cv2.medianBlur(depth, 3) * 1e-3

            # Do instance segmentation
            box, segmentation, mask = self.model(color, 0.5, 0, True, (0,))
            
            # Calculate the target's position
            if box.shape[0]:
                sin, cos = np.sin(yaw), np.cos(yaw)
                d = np.mean(depth[np.logical_and(mask[0], depth != 0)])
                dx, _, dz = d * np.dot(intrinsics, np.array([x, y, 1]).T).T
                odom.pose.pose.position.x = x + dz * cos + dx * sin
                odom.pose.pose.position.y = y + dz * sin - dx * cos
                odom.pose.pose.position.z = z
            self.track.publish(odom)

            # Filter out the target in the depth map
            if segmentation:
                depth[cv2.dilate(
                    0xFF * mask[0].astype(np.uint8), 
                    cv2.getStructuringElement(cv2.MORPH_CROSS, (30,) * 2)
                ) == 0xFF] = 0
            depth[depth > self.d_max] = 0
            
            # Depth -> PointCloud
            h, w = np.mgrid[:depth.shape[0], :depth.shape[1]]
            x = (w - self.sense.intrinsic[0, 2]) * depth
            y = (h - self.sense.intrinsic[1, 2]) * depth
            cloud = np.dstack((
                depth, 
                -x / self.sense.intrinsic[0, 0], 
                -y / self.sense.intrinsic[1, 1]
            ))[::10, ::10, :].reshape(-1, 3)
            self.depth.publish(point_cloud2.create_cloud_xyz32(
                header, cloud[depth[::10, ::10].flatten() != 0].tolist()
            ))

            # Visualize instance segmentation result 
            self.image.publish(self.np2im(self.model.visualize(
                color, box, segmentation
            ), "bgr8"))

            self.sleep()
            
        self.sense.close()


if __name__ == "__main__":
    Camera("camera", "yolov8n-seg.onnx").run()
