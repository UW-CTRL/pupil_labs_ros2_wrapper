#!/home/kysh/venv/pupil_labs/bin/python

# import sys
# print(sys.version)

import array
import time
import traceback

import av
import numpy as np
import rclpy

# import pupil_labs
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from pupil_labs.realtime_api.simple import discover_one_device
from egocentric_msg.msg import GazeData
from sensor_msgs.msg import CompressedImage, Image


def populate_image_message(pl_image_msg, timestamp, frame_id):
    ros_img = Image()
    ros_img.header.stamp.sec = timestamp.sec
    ros_img.header.stamp.nanosec = timestamp.nanosec
    ros_img.header.frame_id = frame_id
    ros_img.height = pl_image_msg.bgr_pixels.shape[0]
    ros_img.width = pl_image_msg.bgr_pixels.shape[1]
    # Bytes per row; viewers and cv_bridge reject images with step = 0
    ros_img.step = ros_img.width * 3
    # Assign an array.array: rclpy checks a bytes value one byte at a time in
    # Python, which takes ~750 ms for a scene frame and capped publishing at ~1 Hz.
    # Copy straight from the pixel buffer, not via tobytes(), to copy only once.
    data = array.array('B')
    data.frombytes(np.ascontiguousarray(pl_image_msg.bgr_pixels))
    ros_img.data = data
    # Set the encoding (e.g., "bgr8" for OpenCV BGR images)
    ros_img.encoding = 'bgr8'  # or "rgb8" depending on your image format
    return ros_img


def populate_compressed_image_message(jpeg, timestamp, frame_id):
    msg = CompressedImage()
    msg.header.stamp.sec = timestamp.sec
    msg.header.stamp.nanosec = timestamp.nanosec
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    data = array.array('B')
    data.frombytes(jpeg)
    msg.data = data
    return msg


def populate_sensor_message(pl_gaze_msg, timestamp):
    msg = GazeData()
    # msg.header.stamp.sec = timestamp.sec
    # msg.header.stamp.nanosec = timestamp.nanosec
    msg.x = pl_gaze_msg.x
    msg.y = pl_gaze_msg.y
    msg.worn = pl_gaze_msg.worn
    msg.pupil_diameter_left = pl_gaze_msg.pupil_diameter_left
    msg.eyeball_center_left_x = pl_gaze_msg.eyeball_center_left_x
    msg.eyeball_center_left_y = pl_gaze_msg.eyeball_center_left_y
    msg.eyeball_center_left_z = pl_gaze_msg.eyeball_center_left_z
    msg.optical_axis_left_x = pl_gaze_msg.optical_axis_left_x
    msg.optical_axis_left_y = pl_gaze_msg.optical_axis_left_y
    msg.optical_axis_left_z = pl_gaze_msg.optical_axis_left_z
    msg.pupil_diameter_right = pl_gaze_msg.pupil_diameter_right
    msg.eyeball_center_right_x = pl_gaze_msg.eyeball_center_right_x
    msg.eyeball_center_right_y = pl_gaze_msg.eyeball_center_right_y
    msg.eyeball_center_right_z = pl_gaze_msg.eyeball_center_right_z
    msg.optical_axis_right_x = pl_gaze_msg.optical_axis_right_x
    msg.optical_axis_right_y = pl_gaze_msg.optical_axis_right_y
    msg.optical_axis_right_z = pl_gaze_msg.optical_axis_right_z
    msg.timestamp_unix_seconds = pl_gaze_msg.timestamp_unix_seconds
    # print(msg)
    return msg


class PupilLabsWrapper(Node):
    def __init__(self):
        super().__init__('pupil_labs_wrapper')
        self.get_logger().info('Looking for the next best device...')
        self.device = discover_one_device(max_search_duration_seconds=10)
        if self.device is None:
            self.get_logger().error('No device found.')
            raise SystemExit(-1)

        self.get_logger().info(f'Connecting to {self.device}...')

        # Setup publishers
        self.pub_gaze = self.create_publisher(GazeData, 'pupil_labs/gaze', 10)
        self.pub_rgb = self.create_publisher(Image, 'pupil_labs/scene_img', 10)
        self.pub_rgb_compressed = self.create_publisher(
            CompressedImage, 'pupil_labs/scene_img/compressed', 10
        )
        # Created on the first frame, once the scene image size is known
        self.jpeg_encoder = None
        self.pub_eyes = self.create_publisher(Image, 'pupil_labs/eye_img', 10)

        self.got_first_frame = False
        self.frame_count = 0
        self.last_report_time = time.monotonic()
        self.get_logger().info('Waiting for first matched scene/eye/gaze frame...')

        self.timer = self.create_timer(1.0 / 30.0, self.publish_pupil_labs_data)

    def publish_pupil_labs_data(self):
        try:
            # Get data from device
            # Time out instead of blocking the executor forever if the stream stalls
            pupil_labs_msg = self.device.receive_matched_scene_and_eyes_video_frames_and_gaze(
                timeout_seconds=1.0
            )
            if pupil_labs_msg is None:
                self.get_logger().warn(
                    'No matched scene/eye/gaze frame in the last second',
                    throttle_duration_sec=5.0,
                )
                return
            # Get current time for timestamp
            current_time = self.get_clock().now().to_msg()
            # Populate and publish messages
            self.pub_gaze.publish(populate_sensor_message(pupil_labs_msg.gaze, current_time))
            self.pub_rgb.publish(
                populate_image_message(pupil_labs_msg.scene, current_time, 'pupil_scene')
            )
            self.pub_eyes.publish(
                populate_image_message(pupil_labs_msg.eyes, current_time, 'pupil_eyes')
            )
            # JPEG encoding takes ~20 ms per frame, so skip it when nobody listens
            if self.pub_rgb_compressed.get_subscription_count() > 0:
                self.pub_rgb_compressed.publish(
                    populate_compressed_image_message(
                        self.encode_jpeg(pupil_labs_msg.scene.bgr_pixels),
                        current_time,
                        'pupil_scene',
                    )
                )
            self.report_progress(pupil_labs_msg)
        except Exception as e:
            self.get_logger().error(
                f'Error receiving or publishing data: {e}\n{traceback.format_exc()}',
                throttle_duration_sec=2.0,
            )

    def encode_jpeg(self, bgr_pixels):
        height, width = bgr_pixels.shape[:2]
        if self.jpeg_encoder is None:
            # OpenCV can't be imported alongside the NumPy 2 the realtime API
            # needs, so encode with PyAV, which the realtime API depends on
            self.jpeg_encoder = av.CodecContext.create('mjpeg', 'w')
            self.jpeg_encoder.width = width
            self.jpeg_encoder.height = height
            self.jpeg_encoder.pix_fmt = 'yuvj420p'
            # 2 (best) to 31 (worst); 3 is about JPEG quality 90
            self.jpeg_encoder.options = {'qscale': '3'}
        frame = av.VideoFrame.from_ndarray(bgr_pixels, format='bgr24')
        packets = self.jpeg_encoder.encode(frame.reformat(format='yuvj420p'))
        return b''.join(bytes(packet) for packet in packets)

    def report_progress(self, pupil_labs_msg):
        self.frame_count += 1
        if not self.got_first_frame:
            self.got_first_frame = True
            self.get_logger().info(
                f'First frame received: scene {pupil_labs_msg.scene.bgr_pixels.shape}, '
                f'eyes {pupil_labs_msg.eyes.bgr_pixels.shape}'
            )
        now = time.monotonic()
        elapsed = now - self.last_report_time
        if elapsed >= 5.0:
            self.get_logger().info(f'Publishing at {self.frame_count / elapsed:.1f} Hz')
            self.frame_count = 0
            self.last_report_time = now

    def destroy_node(self):
        # The device streams on a non-daemon thread, which keeps the process
        # alive after shutdown unless the device is closed
        self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PupilLabsWrapper()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
