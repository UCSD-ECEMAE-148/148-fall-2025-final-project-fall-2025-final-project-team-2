#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response

# Flask app
app = Flask(__name__)

latest_frame = None  # global buffer


class OakViewerNode(Node):
    def __init__(self):
        super().__init__('oak_viewer_node')

        self.bridge = CvBridge()

        # Subscribe to the annotated camera topic
        self.create_subscription(
            Image,
            'oak_annotated',
            self.image_callback,
            10
        )

        self.get_logger().info("OAK Viewer Node started. Listening to /oak_annotated")

    def image_callback(self, msg):
        global latest_frame
        try:
            frame = self.bridge.cv2_to_imgmsg(msg)
        except:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        latest_frame = frame


def generate_frames():
    global latest_frame
    while True:
        if latest_frame is None:
            continue

        ret, jpeg = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue

        frame_bytes = jpeg.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame_bytes +
            b'\r\n'
        )


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return "<h1>OAK Camera Feed</h1><img src='/video_feed' width='640'>"


def main():
    rclpy.init()

    node = OakViewerNode()

    # Run Flask in a separate thread
    import threading
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)
    )
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
