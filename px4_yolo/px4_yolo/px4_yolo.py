#!/usr/bin/env python3
# yolo_ros2_node.py
"""
ROS2 node:
 - subscribes to an Image topic
 - runs YOLOv8 inference using ultralytics (custom model .pt)
 - publishes annotated image (sensor_msgs/Image)
 - publishes detections as JSON on std_msgs/String for easy parsing

Parameters (ros2 params / launch):
 - image_topic (string)             : input image topic (ROS image)
 - output_image_topic (string)      : annotated image topic
 - detections_topic (string)        : detections JSON topic
 - model_path (string)              : path to your best.pt (custom model)
 - conf_threshold (float)           : confidence threshold (0-1)
 - imgsz (int)                      : inference image size (square)
 - device (string)                  : 'cpu' or '0' (GPU id)
 - class_names (list of strings)    : optional fallback list of class names
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import json
import threading
import os
import time

# Try to import ultralytics
USE_ULTRALYTICS = False
try:
    from ultralytics import YOLO
    USE_ULTRALYTICS = True
except Exception as e:
    USE_ULTRALYTICS = False

import cv2


class YoloRos2Node(Node):
    def __init__(self):
        super().__init__('yolo_ros2_node')

        # declare params (defaults tuned for your setup)
        self.declare_parameter('image_topic', '/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image')
        self.declare_parameter('output_image_topic', '/drone1/camera/image_detected')
        self.declare_parameter('detections_topic', '/drone1/camera/detections_json')
        self.declare_parameter('model_path', '/home/het/models/best.pt')  # change to your path
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')  # or '0' for GPU
        self.declare_parameter('class_names', r"['Automatic Rifle','Bazooka','Grenade Launcher','Handgun','Knife','Shotgun','SMG','Sniper','Sword']")

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = float(self.get_parameter('conf_threshold').get_parameter_value().double_value)
        self.imgsz = int(self.get_parameter('imgsz').get_parameter_value().integer_value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        # parse class_names param (string) into python list if needed
        raw_class_names = self.get_parameter('class_names').get_parameter_value().string_value
        try:
            # try to evaluate a Python list literal safely
            class_names = eval(raw_class_names) if isinstance(raw_class_names, str) else raw_class_names
            if isinstance(class_names, (list, tuple)):
                self.class_names = list(class_names)
            else:
                self.class_names = []
        except Exception:
            self.class_names = []

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # publishers & subscriber
        self.pub_img = self.create_publisher(Image, self.output_image_topic, 1)
        self.pub_det = self.create_publisher(String, self.detections_topic, 1)
        self.sub_img = self.create_subscription(Image, self.image_topic, self.image_cb, 1)

        # load model
        if not USE_ULTRALYTICS:
            self.get_logger().error('Ultralytics YOLO (ultralytics package) not found. Install via `pip install ultralytics`.')
            raise RuntimeError('ultralytics not installed')

        if not os.path.exists(self.model_path):
            # ultralytics can accept shorthand like 'yolov8n.pt' and auto-download; but for custom model path we expect a real file
            self.get_logger().warn(f'Model path {self.model_path} not found on disk. If you intended a remote model name (e.g. yolov8n.pt), ensure ultralytics can download it, or place your custom best.pt at the path.')
        self.get_logger().info(f'Loading model: {self.model_path} (device={self.device})')
        try:
            self.model = YOLO(self.model_path)
            # if model contains .names or .model.names, ultralytics will expose them
            try:
                # ultralytics may store names in model.model.names or model.names
                names = getattr(self.model, 'names', None)
                if names is None and hasattr(self.model, 'model') and hasattr(self.model.model, 'names'):
                    names = self.model.model.names
                if names:
                    # names can be dict index->name
                    if isinstance(names, dict):
                        # convert dict to list
                        self.class_names = [names[i] for i in range(len(names))]
                    elif isinstance(names, (list, tuple)):
                        self.class_names = list(names)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        if not self.class_names:
            self.get_logger().warn('No class names found inside model or params. Provide `class_names` param or ensure model has .names.')
        else:
            self.get_logger().info(f'Loaded class names ({len(self.class_names)}): {self.class_names}')

        self.get_logger().info(f'YoloRos2Node ready. Subscribing: {self.image_topic} -> publishing annotated: {self.output_image_topic} and detections: {self.detections_topic}')

    def image_cb(self, msg: Image):
        # non-blocking: skip frame if processing
        if not self.lock.acquire(False):
            # drop frame
            return
        try:
            # convert ROS Image -> cv2
            try:
                # inspect encoding and convert safely
                enc = msg.encoding.lower() if hasattr(msg, 'encoding') else 'bgr8'
                if 'rgb' in enc:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                    # convert to BGR for OpenCV
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else:
                    # treat as bgr8 by default
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'cv_bridge conversion failed: {e}')
                return

            h, w = cv_img.shape[:2]
            t0 = time.time()

            # run inference
            try:
                # ultralytics supports calling the model with numpy images
                results = self.model(cv_img, imgsz=self.imgsz, conf=self.conf_threshold, device=self.device)
                r = results[0]
                # r.boxes, r.boxes.xyxy, r.boxes.conf, r.boxes.cls
                # construct annotated image (copy)
                annotated = cv_img.copy()

                detections_list = []
                # ultralytics may provide xyxy as tensor or numpy
                boxes_xyxy = []
                scores = []
                classes = []

                # safe access: r.boxes may be object with attributes
                if hasattr(r, 'boxes') and r.boxes is not None:
                    try:
                        xyxy = r.boxes.xyxy.cpu().numpy() if hasattr(r.boxes.xyxy, 'cpu') else np.array(r.boxes.xyxy)
                        confs = r.boxes.conf.cpu().numpy() if hasattr(r.boxes.conf, 'cpu') else np.array(r.boxes.conf)
                        clsids = r.boxes.cls.cpu().numpy() if hasattr(r.boxes.cls, 'cpu') else np.array(r.boxes.cls)
                        boxes_xyxy = xyxy
                        scores = confs
                        classes = clsids
                    except Exception:
                        # fallbacks to attributes in result
                        try:
                            for box in r.boxes:
                                b = box.xyxy.tolist()
                                c = float(box.conf)
                                cl = int(box.cls)
                                boxes_xyxy.append(b)
                                scores.append(c)
                                classes.append(cl)
                        except Exception:
                            pass

                # draw boxes
                for i, box in enumerate(boxes_xyxy):
                    try:
                        x1, y1, x2, y2 = [int(round(v)) for v in box]
                        score = float(scores[i]) if i < len(scores) else 0.0
                        clsid = int(classes[i]) if i < len(classes) else -1
                        label = str(clsid)
                        if 0 <= clsid < len(self.class_names):
                            label = self.class_names[clsid]
                        # draw rectangle and label
                        color = (0, 255, 0)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                        txt = f'{label} {score:.2f}'
                        cv2.putText(annotated, txt, (max(x1, 0), max(y1-6, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                        # append to detections list (normalized bbox and pixel bbox)
                        bbox_px = {'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2)}
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0
                        wnorm = (x2 - x1) / float(w)
                        hnorm = (y2 - y1) / float(h)
                        detections_list.append({
                            'class_id': int(clsid),
                            'class_name': label,
                            'score': float(score),
                            'bbox_px': bbox_px,
                            'bbox_norm': {'cx': cx / float(w), 'cy': cy / float(h), 'w': wnorm, 'h': hnorm}
                        })
                    except Exception as e:
                        self.get_logger().warn(f'Failed to process one detection: {e}')

                # publish annotated image
                try:
                    out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                    out_msg.header = msg.header
                    self.pub_img.publish(out_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish annotated image: {e}')

                # publish detections as JSON string
                try:
                    payload = {
                        'timestamp': self.get_clock().now().to_msg().sec,
                        'source_topic': self.image_topic,
                        'image_width': w,
                        'image_height': h,
                        'detections': detections_list
                    }
                    s = String()
                    s.data = json.dumps(payload)
                    self.pub_det.publish(s)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish detections JSON: {e}')

            except Exception as e:
                self.get_logger().error(f'Inference error: {e}')
                # publish empty detections and original image to keep pipeline alive
                try:
                    out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                    out_msg.header = msg.header
                    self.pub_img.publish(out_msg)
                    payload = {'timestamp': self.get_clock().now().to_msg().sec, 'source_topic': self.image_topic, 'detections': []}
                    self.pub_det.publish(String(data=json.dumps(payload)))
                except Exception:
                    pass

            t1 = time.time()
            self.get_logger().debug(f'Processed frame in {1000*(t1-t0):.1f} ms -- detections: {len(detections_list)}')
        finally:
            self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloRos2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print('Node failed:', e)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
