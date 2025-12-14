
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32

import numpy as np
import depthai as dai

# Use your existing logic + summit code from the same package
from .fire_detection_logic import process_frame
from .summit import build_or_update_summits_from_matrices, choisir_meilleure_cible


class FireDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('fire_detection_node')

        # ---------------- Publishers ----------------
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

        # ---------------- Summit decision state ----------------
        self.summits_by_id = None
        self.current_window_id = 4  # can start at 0 or whatever you want

        # ---------------- DepthAI pipeline ----------------
        self.pipeline = dai.Pipeline()

        cam = self.pipeline.createColorCamera()
        cam.setPreviewSize(640, 480)
        cam.setInterleaved(False)

        xout = self.pipeline.createXLinkOut()
        xout.setStreamName('video')
        cam.preview.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.queue = self.device.getOutputQueue(
            name='video',
            maxSize=1,
            blocking=False,
        )

        # Timer ~10 Hz
        self.timer = self.create_timer(0.1, self.process_image)

        self.get_logger().info('FireDetectionNode started.')

    # ---------------- Main timer callback ----------------
    def process_image(self) -> None:
        frame_msg = self.queue.tryGet()
        if frame_msg is None:
            return

        frame = frame_msg.getCvFrame()

        (
            annotated,
            temps,
            persons,
            windows,
            temp_matrix,
            person_matrix,
        ) = process_frame(frame)
        self.get_logger().info(f"Detected {len(windows)} windows.")
        # ---------- Publish matrix shape ----------
        rows, cols = temp_matrix.shape
        shape_msg = Int32MultiArray()
        shape_msg.data = [rows, cols]
        self.shape_pub.publish(shape_msg)

        # ---------- Publish temperature matrix ----------
        temp_msg = Float32MultiArray()
        temp_msg.data = temp_matrix.flatten().tolist()
        self.temp_pub.publish(temp_msg)

        # ---------- Publish person matrix ----------
        person_msg = Int32MultiArray()
        person_msg.data = person_matrix.flatten().tolist()
        self.person_pub.publish(person_msg)

        # ---------- Your summit decision logic ----------
        table_sommets, self.summits_by_id, id_matrix = \
            build_or_update_summits_from_matrices(
                temp_matrix,
                person_matrix,
                existing_summits=self.summits_by_id,
            )

        if table_sommets:
            # make sure current position is valid
            if self.current_window_id not in self.summits_by_id:
                self.current_window_id = table_sommets[0].id

            meilleure_cible = choisir_meilleure_cible(
                self.current_window_id,
                table_sommets,
            )

            if meilleure_cible is not None:
                self.current_window_id = meilleure_cible.id
                
                # Publish the best window id
                target_msg = Int32()
                target_msg.data = int(meilleure_cible.id)
                self.best_target_pub.publish(target_msg)

                self.get_logger().info(
                    f'Best target window id: {meilleure_cible.id}'
                )

    def destroy_node(self) -> None:
        # try to cleanly close the DepthAI device
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
