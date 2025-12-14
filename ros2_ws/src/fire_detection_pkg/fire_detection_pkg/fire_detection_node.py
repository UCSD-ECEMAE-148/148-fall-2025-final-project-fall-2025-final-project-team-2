#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import depthai as dai

from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .fire_detection_logic import process_frame
from .summit import build_or_update_summits_from_matrices, choisir_meilleure_cible


class FireDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('fire_detection_node')

        # ------------- Publishers -------------
        self.temp_pub = self.create_publisher(
            Float32MultiArray, 'temp_matrix', 10
        )
        self.person_pub = self.create_publisher(
            Int32MultiArray, 'person_matrix', 10
        )
        self.shape_pub = self.create_publisher(
            Int32MultiArray, 'matrix_shape', 10
        )
        self.best_target_pub = self.create_publisher(
            Int32, 'best_target_window', 10
        )
        self.image_pub = self.create_publisher(
            Image, 'oak_annotated', 10
        )

        self.bridge = CvBridge()

        # ------------- Cycle lock (wait for BrainNode) -------------
        # When we publish a best target, we wait until BrainNode sends /cycle_complete
        self.waiting_for_brain = False
        self.cycle_sub = self.create_subscription(
            Bool,
            'cycle_complete',
            self.on_cycle_complete,
            10
        )

        # ------------- Summit state -------------
        self.summits_by_id = None
        # Using 0–8 indexing; start in center (4)
        self.current_window_id = 4

        # ------------- DepthAI pipeline -------------
        self.pipeline = dai.Pipeline()

        cam = self.pipeline.createColorCamera()
        cam.setVideoSize(1280, 720)     # force 640 x 480 like your laptop
        cam.setInterleaved(False)
        cam.setFps(30)
        xout = self.pipeline.createXLinkOut()
        xout.setStreamName('video')
        cam.video.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(
            name='video',
            maxSize=1,
            blocking=False
        )

        # 10 Hz processing timer
        self.timer = self.create_timer(0.1, self.process_image)
       
        self.get_logger().info('FireDetectionNode started.')

    # ------------- callback: BrainNode finished a cycle -------------
    def on_cycle_complete(self, msg: Bool) -> None:
        if msg.data:
            self.waiting_for_brain = False
            self.get_logger().info('cycle_complete received → unlocked for next target.')

    # ------------- main processing loop -------------
    def process_image(self) -> None:
        # If we already sent a target and are waiting for BrainNode, do nothing
        if self.waiting_for_brain:
            return

        frame_msg = self.queue.tryGet()
        if frame_msg is None:
            return

        frame = frame_msg.getCvFrame()
        if frame is None:
            return

        # Just for debugging resolution
        self.get_logger().info(f"Frame shape: {frame.shape}")

        (
            annotated,
            temps_flat,
            persons_flat,
            windows,
            temp_matrix,
            person_matrix,
        ) = process_frame(frame)

        # --------- Publish annotated image ---------
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

        # --------- Publish matrix shape (always 3x3 now) ---------
        rows, cols = temp_matrix.shape
        shape_msg = Int32MultiArray()
        shape_msg.data = [rows, cols]
        self.shape_pub.publish(shape_msg)

        # --------- Publish temperature matrix ---------
        temp_msg = Float32MultiArray()
        temp_msg.data = temp_matrix.flatten().tolist()
        self.temp_pub.publish(temp_msg)

        # --------- Publish person matrix ---------
        person_msg = Int32MultiArray()
        person_msg.data = person_matrix.flatten().tolist()
        self.person_pub.publish(person_msg)

        self.get_logger().info(
            f"Detected windows (non-NaN): "
            f"{np.count_nonzero(~np.isnan(temp_matrix))}"
        )

        # --------- Summit decision logic ---------
        table_sommets, self.summits_by_id, id_matrix = build_or_update_summits_from_matrices(
            temp_matrix,
            person_matrix,
            existing_summits=self.summits_by_id,
        )

        if table_sommets:
            # ensure current_window_id is valid
            if self.current_window_id not in self.summits_by_id:
                self.current_window_id = table_sommets[0].id

            meilleure_cible = choisir_meilleure_cible(
                self.current_window_id,
                table_sommets,
            )

            if meilleure_cible is not None:
                self.current_window_id = int(meilleure_cible.id)

                target_msg = Int32()
                target_msg.data = self.current_window_id
                self.best_target_pub.publish(target_msg)

                self.get_logger().info(
                    f"Best target window id (0–8, one per cycle): {self.current_window_id}"
                )

                # lock until BrainNode sends /cycle_complete
                self.waiting_for_brain = True

    # ------------- cleanup -------------
    def destroy_node(self) -> None:
        try:
            self.device.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
