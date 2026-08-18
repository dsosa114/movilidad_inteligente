import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy


class LaneHlsTuner(Node):
    def __init__(self):
        super().__init__('lane_hls_tuner')

        self.declare_parameter('target_node', '/lane_detector')
        self.declare_parameter('window_name', 'Lane HLS Tuner')
        self.declare_parameter('push_rate_hz', 5.0)
        self.declare_parameter('subscribe_image_topic', '')
        self.declare_parameter('push_on_change', False)

        self.target_node = self.get_parameter('target_node').value
        self.window_name = self.get_parameter('window_name').value
        push_rate_hz = max(1.0, float(self.get_parameter('push_rate_hz').value))
        self.push_on_change = bool(self.get_parameter('push_on_change').value)
        self.subscribe_image_topic = str(self.get_parameter('subscribe_image_topic').value)

        self.param_client = AsyncParameterClient(self, self.target_node)
        self.br = CvBridge()
        self.subscription = None
        self.latest_frame_bgr = None

        self.qcar_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._ignore_trackbar_events = False
        self._dirty = True
        self._synced_from_target = False
        self._pending_get_future = None

        cv2.namedWindow(self.window_name)
        cv2.namedWindow('Lane HLS Preview')
        self._create_trackbars()

        if self.subscribe_image_topic:
            self._create_image_subscription(self.subscribe_image_topic)

        self.timer = self.create_timer(1.0 / push_rate_hz, self._timer_callback)
        self.get_logger().info(
            f'Lane HLS tuner started. Target node: {self.target_node}. '
            'Move trackbars to preview masks. Press u to push parameters, q to quit.'
        )

    def _defaults(self):
        return {
            'gray_lower_hls': [30, 160, 0],
            'gray_upper_hls': [180, 200, 40],
            'yellow_lower_hls': [15, 30, 115],
            'yellow_upper_hls': [35, 204, 255],
        }

    def _create_trackbars(self):
        d = self._defaults()

        # Gray trackbars
        cv2.createTrackbar('Gray H Low', self.window_name, d['gray_lower_hls'][0], 180, self._on_slider)
        cv2.createTrackbar('Gray L Low', self.window_name, d['gray_lower_hls'][1], 255, self._on_slider)
        cv2.createTrackbar('Gray S Low', self.window_name, d['gray_lower_hls'][2], 255, self._on_slider)
        cv2.createTrackbar('Gray H High', self.window_name, d['gray_upper_hls'][0], 180, self._on_slider)
        cv2.createTrackbar('Gray L High', self.window_name, d['gray_upper_hls'][1], 255, self._on_slider)
        cv2.createTrackbar('Gray S High', self.window_name, d['gray_upper_hls'][2], 255, self._on_slider)

        # Yellow trackbars
        cv2.createTrackbar('Yellow H Low', self.window_name, d['yellow_lower_hls'][0], 180, self._on_slider)
        cv2.createTrackbar('Yellow L Low', self.window_name, d['yellow_lower_hls'][1], 255, self._on_slider)
        cv2.createTrackbar('Yellow S Low', self.window_name, d['yellow_lower_hls'][2], 255, self._on_slider)
        cv2.createTrackbar('Yellow H High', self.window_name, d['yellow_upper_hls'][0], 180, self._on_slider)
        cv2.createTrackbar('Yellow L High', self.window_name, d['yellow_upper_hls'][1], 255, self._on_slider)
        cv2.createTrackbar('Yellow S High', self.window_name, d['yellow_upper_hls'][2], 255, self._on_slider)

    def _set_trackbar_values(self, values):
        self._ignore_trackbar_events = True
        try:
            cv2.setTrackbarPos('Gray H Low', self.window_name, int(values['gray_lower_hls'][0]))
            cv2.setTrackbarPos('Gray L Low', self.window_name, int(values['gray_lower_hls'][1]))
            cv2.setTrackbarPos('Gray S Low', self.window_name, int(values['gray_lower_hls'][2]))
            cv2.setTrackbarPos('Gray H High', self.window_name, int(values['gray_upper_hls'][0]))
            cv2.setTrackbarPos('Gray L High', self.window_name, int(values['gray_upper_hls'][1]))
            cv2.setTrackbarPos('Gray S High', self.window_name, int(values['gray_upper_hls'][2]))

            cv2.setTrackbarPos('Yellow H Low', self.window_name, int(values['yellow_lower_hls'][0]))
            cv2.setTrackbarPos('Yellow L Low', self.window_name, int(values['yellow_lower_hls'][1]))
            cv2.setTrackbarPos('Yellow S Low', self.window_name, int(values['yellow_lower_hls'][2]))
            cv2.setTrackbarPos('Yellow H High', self.window_name, int(values['yellow_upper_hls'][0]))
            cv2.setTrackbarPos('Yellow L High', self.window_name, int(values['yellow_upper_hls'][1]))
            cv2.setTrackbarPos('Yellow S High', self.window_name, int(values['yellow_upper_hls'][2]))
        finally:
            self._ignore_trackbar_events = False

    def _on_slider(self, _):
        if not self._ignore_trackbar_events:
            self._dirty = True

    def _read_trackbar_values(self):
        values = {
            'gray_lower_hls': [
                cv2.getTrackbarPos('Gray H Low', self.window_name),
                cv2.getTrackbarPos('Gray L Low', self.window_name),
                cv2.getTrackbarPos('Gray S Low', self.window_name),
            ],
            'gray_upper_hls': [
                cv2.getTrackbarPos('Gray H High', self.window_name),
                cv2.getTrackbarPos('Gray L High', self.window_name),
                cv2.getTrackbarPos('Gray S High', self.window_name),
            ],
            'yellow_lower_hls': [
                cv2.getTrackbarPos('Yellow H Low', self.window_name),
                cv2.getTrackbarPos('Yellow L Low', self.window_name),
                cv2.getTrackbarPos('Yellow S Low', self.window_name),
            ],
            'yellow_upper_hls': [
                cv2.getTrackbarPos('Yellow H High', self.window_name),
                cv2.getTrackbarPos('Yellow L High', self.window_name),
                cv2.getTrackbarPos('Yellow S High', self.window_name),
            ],
        }
        return values

    def _create_image_subscription(self, topic_name):
        if self.subscription is not None:
            return
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self._image_callback,
            self.qcar_qos_profile,
        )
        self.get_logger().info(f'Subscribed to image topic: {topic_name}')

    def _image_callback(self, msg):
        try:
            self.latest_frame_bgr = self.br.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Failed to decode image frame: {exc}')

    def _color_segment(self, hls, lower_range, upper_range):
        lower = np.asarray(lower_range, dtype=np.uint8).reshape(-1)
        upper = np.asarray(upper_range, dtype=np.uint8).reshape(-1)
        mask_in_range = cv2.inRange(hls, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        return cv2.morphologyEx(mask_in_range, cv2.MORPH_DILATE, kernel)

    def _render_preview(self):
        if self.latest_frame_bgr is None:
            return

        values = self._read_trackbar_values()
        frame = self.latest_frame_bgr
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

        gray_mask = self._color_segment(hls, values['gray_lower_hls'], values['gray_upper_hls'])
        yellow_mask = self._color_segment(hls, values['yellow_lower_hls'], values['yellow_upper_hls'])
        combined_mask = cv2.bitwise_or(gray_mask, yellow_mask)

        gray_viz = cv2.cvtColor(gray_mask, cv2.COLOR_GRAY2BGR)
        yellow_viz = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
        combined_viz = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.bitwise_and(frame, combined_viz)

        top_row = np.hstack((frame, overlay))
        bottom_row = np.hstack((gray_viz, yellow_viz))
        preview = np.vstack((top_row, bottom_row))

        cv2.putText(preview, 'Original', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(preview, 'Combined Overlay', (frame.shape[1] + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(preview, 'Gray Mask', (10, frame.shape[0] + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(preview, 'Yellow Mask', (frame.shape[1] + 10, frame.shape[0] + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Lane HLS Preview', preview)

    def _sync_from_target_once(self):
        if self._synced_from_target:
            return
        if not self.param_client.services_are_ready():
            return

        if self._pending_get_future is None:
            self._pending_get_future = self.param_client.get_parameters([
                'subscribe_image_topic',
                'gray_lower_hls',
                'gray_upper_hls',
                'yellow_lower_hls',
                'yellow_upper_hls',
            ])
            return

        if not self._pending_get_future.done():
            return

        try:
            result = self._pending_get_future.result()
            params = result.values
            if len(params) == 5:
                if not self.subscribe_image_topic:
                    subscribed_topic = params[0].string_value
                else:
                    subscribed_topic = self.subscribe_image_topic
                if subscribed_topic and self.subscription is None:
                    self._create_image_subscription(subscribed_topic)

                values = {
                    'gray_lower_hls': list(params[1].integer_array_value),
                    'gray_upper_hls': list(params[2].integer_array_value),
                    'yellow_lower_hls': list(params[3].integer_array_value),
                    'yellow_upper_hls': list(params[4].integer_array_value),
                }
                if all(len(v) == 3 for v in values.values()):
                    self._set_trackbar_values(values)
                    self._dirty = False
                    self.get_logger().info('Trackbars initialized from lane_detector current parameters.')
        except Exception as exc:
            self.get_logger().warning(f'Failed to read current parameters from {self.target_node}: {exc}')

        self._synced_from_target = True
        self._pending_get_future = None

    def _push_values(self, values):
        params = [
            Parameter('gray_lower_hls', value=values['gray_lower_hls']),
            Parameter('gray_upper_hls', value=values['gray_upper_hls']),
            Parameter('yellow_lower_hls', value=values['yellow_lower_hls']),
            Parameter('yellow_upper_hls', value=values['yellow_upper_hls']),
        ]

        future = self.param_client.set_parameters(params)

        def _done_cb(done_future):
            try:
                results = done_future.result()
                all_ok = all(r.successful for r in results)
                if not all_ok:
                    reasons = [r.reason for r in results if not r.successful]
                    self.get_logger().warning(
                        f'Parameter update rejected by {self.target_node}: {reasons}'
                    )
            except Exception as exc:
                self.get_logger().warning(f'Failed to push parameters to {self.target_node}: {exc}')

        future.add_done_callback(_done_cb)

    def _timer_callback(self):
        # Keep HighGUI responsive
        key = cv2.waitKey(1) & 0xFF
        if key == ord('u'):
            self._dirty = True
            self.get_logger().info('Sending current trackbar values to lane_detector...')
        elif key == ord('q'):
            self.get_logger().info('Quit requested from tuner window.')
            self.destroy_node()
            cv2.destroyAllWindows()
            return

        self._render_preview()

        self._sync_from_target_once()

        if not self._dirty:
            return

        if not self.param_client.services_are_ready():
            return

        if not self.push_on_change and key != ord('u'):
            return

        values = self._read_trackbar_values()
        self._push_values(values)
        self._dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = LaneHlsTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
