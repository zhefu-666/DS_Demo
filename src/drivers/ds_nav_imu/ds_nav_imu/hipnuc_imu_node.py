#!/usr/bin/env python3

import math
import struct
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField
import serial


SOF = b"\x5a\xa5"
MAX_PAYLOAD_LEN = 512
HI91_TAG = 0x91
HI91_PAYLOAD_LEN = 76
STANDARD_GRAVITY = 9.80665
DEVICE_TIME_MODULUS_MS = 1 << 32


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


class HipnucImuNode(Node):
    def __init__(self):
        super().__init__("hipnuc_imu_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("mag_topic", "/imu/mag")
        self.declare_parameter("publish_magnetic_field", True)
        self.declare_parameter("orientation_variance", 0.01)
        self.declare_parameter("angular_velocity_variance", 0.01)
        self.declare_parameter("linear_acceleration_variance", 0.1)
        self.declare_parameter("magnetic_field_variance", 1.0e-6)
        self.declare_parameter("timestamp_source", "host_receive")
        self.declare_parameter("timestamp_offset_sec", 0.0)

        self.serial_port = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_magnetic_field = bool(
            self.get_parameter("publish_magnetic_field").value
        )
        self.timestamp_source = str(self.get_parameter("timestamp_source").value)
        if self.timestamp_source not in ("host_receive", "device"):
            raise ValueError("timestamp_source must be 'host_receive' or 'device'")
        self.timestamp_offset = float(
            self.get_parameter("timestamp_offset_sec").value
        )
        self._device_time_last_ms = None
        self._device_time_unwrapped_ms = None
        self._device_time_origin_ms = None
        self._device_time_origin_ros_ns = None

        imu_topic = self.get_parameter("imu_topic").value
        mag_topic = self.get_parameter("mag_topic").value
        self.imu_pub = self.create_publisher(Imu, imu_topic, 20)
        self.mag_pub = (
            self.create_publisher(MagneticField, mag_topic, 20)
            if self.publish_magnetic_field
            else None
        )

        self.orientation_variance = float(
            self.get_parameter("orientation_variance").value
        )
        self.angular_velocity_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        self.linear_acceleration_variance = float(
            self.get_parameter("linear_acceleration_variance").value
        )
        self.magnetic_field_variance = float(
            self.get_parameter("magnetic_field_variance").value
        )

        self.buffer = bytearray()
        self.running = True
        self.serial = serial.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )

        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()
        self.get_logger().info(
            f"HiPNUC IMU reading {self.serial_port} at {self.baudrate}, "
            f"publishing {imu_topic}"
        )

    def destroy_node(self):
        self.running = False
        if hasattr(self, "reader_thread") and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if hasattr(self, "serial") and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

    def read_loop(self):
        while rclpy.ok() and self.running:
            try:
                # Read one byte at a time so a continuous stream cannot queue
                # dozens of milliseconds of frames before they are stamped.
                chunk = self.serial.read(1)
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read failed: {exc}")
                time.sleep(1.0)
                continue

            if not chunk:
                continue

            # Capture arrival before parsing/decoding so parser latency is not
            # included in the sample timestamp.  With one-byte reads this is
            # the completion time of the current frame, within one byte time.
            receive_time = self.get_clock().now()
            self.buffer.extend(chunk)
            self.parse_buffer(receive_time)

    def parse_buffer(self, receive_time):
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return

            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 6:
                return

            payload_len = struct.unpack_from("<H", self.buffer, 2)[0]
            if payload_len < 1 or payload_len > MAX_PAYLOAD_LEN:
                del self.buffer[0]
                continue

            frame_len = 6 + payload_len
            if len(self.buffer) < frame_len:
                return

            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            expected_crc = struct.unpack_from("<H", frame, 4)[0]
            actual_crc = crc16_xmodem(frame[:4] + frame[6:])
            if expected_crc != actual_crc:
                self.get_logger().warn(
                    "Dropped HiPNUC frame with bad CRC",
                    throttle_duration_sec=5.0,
                )
                continue

            payload = frame[6:]
            if payload_len == HI91_PAYLOAD_LEN and payload[0] == HI91_TAG:
                self.publish_hi91(payload, receive_time)

    def publish_hi91(self, payload: bytes, receive_time):
        (
            _tag,
            _status,
            _temperature,
            _pressure,
            _device_time_ms,
            acc_x_g,
            acc_y_g,
            acc_z_g,
            gyro_x_deg_s,
            gyro_y_deg_s,
            gyro_z_deg_s,
            mag_x_ut,
            mag_y_ut,
            mag_z_ut,
            _roll_deg,
            _pitch_deg,
            _yaw_deg,
            quat_w,
            quat_x,
            quat_y,
            quat_z,
        ) = struct.unpack("<BHb f I 16f", payload)

        stamp = self.sample_time(receive_time, _device_time_ms).to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation.w = float(quat_w)
        imu.orientation.x = float(quat_x)
        imu.orientation.y = float(quat_y)
        imu.orientation.z = float(quat_z)
        imu.angular_velocity.x = math.radians(float(gyro_x_deg_s))
        imu.angular_velocity.y = math.radians(float(gyro_y_deg_s))
        imu.angular_velocity.z = math.radians(float(gyro_z_deg_s))
        imu.linear_acceleration.x = float(acc_x_g) * STANDARD_GRAVITY
        imu.linear_acceleration.y = float(acc_y_g) * STANDARD_GRAVITY
        imu.linear_acceleration.z = float(acc_z_g) * STANDARD_GRAVITY

        self.fill_diagonal(imu.orientation_covariance, self.orientation_variance)
        self.fill_diagonal(
            imu.angular_velocity_covariance, self.angular_velocity_variance
        )
        self.fill_diagonal(
            imu.linear_acceleration_covariance,
            self.linear_acceleration_variance,
        )

        self.imu_pub.publish(imu)

        if self.mag_pub is not None:
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self.frame_id
            mag.magnetic_field.x = float(mag_x_ut) * 1.0e-6
            mag.magnetic_field.y = float(mag_y_ut) * 1.0e-6
            mag.magnetic_field.z = float(mag_z_ut) * 1.0e-6
            self.fill_diagonal(
                mag.magnetic_field_covariance,
                self.magnetic_field_variance,
            )
            self.mag_pub.publish(mag)

    def sample_time(self, receive_time: Time, device_time_ms: int) -> Time:
        """Return an IMU stamp in the ROS time domain used by RealSense.

        ``host_receive`` is the default for devices on the same host.
        ``device`` preserves the HiPNUC clock's short-term sample spacing while
        anchoring its first sample to the host clock.  The configured offset is
        applied in both modes; a negative value generally removes serial delay.
        """
        receive_ns = receive_time.nanoseconds + int(
            Duration(seconds=self.timestamp_offset).nanoseconds
        )
        if self.timestamp_source == "host_receive":
            return Time(nanoseconds=receive_ns, clock_type=receive_time.clock_type)

        # Unwrap the uint32 device millisecond counter across rollover and
        # occasional packet reordering.
        device_ms = int(device_time_ms) & 0xFFFFFFFF
        if self._device_time_last_ms is None:
            self._device_time_last_ms = device_ms
            self._device_time_unwrapped_ms = device_ms
        else:
            delta = (device_ms - self._device_time_last_ms) % DEVICE_TIME_MODULUS_MS
            if delta > DEVICE_TIME_MODULUS_MS // 2:
                delta -= DEVICE_TIME_MODULUS_MS
            if delta >= 0:
                self._device_time_unwrapped_ms += delta
                self._device_time_last_ms = device_ms

        if self._device_time_origin_ms is None:
            self._device_time_origin_ms = self._device_time_unwrapped_ms
            self._device_time_origin_ros_ns = receive_ns

        elapsed_ns = int(round(
            (self._device_time_unwrapped_ms - self._device_time_origin_ms) * 1.0e6
        ))
        return Time(
            nanoseconds=self._device_time_origin_ros_ns + elapsed_ns,
            clock_type=receive_time.clock_type,
        )

    @staticmethod
    def fill_diagonal(covariance, value):
        covariance[0] = value
        covariance[4] = value
        covariance[8] = value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HipnucImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
