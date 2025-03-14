import time
from time import sleep
import numpy as np
import serial
from serial import Serial
from serial.tools.list_ports import comports
import serial.tools.list_ports


class Echo:

    def __init__(
        self,
        vid: int = 1603,
        pid: int = 1868,
        baudrate: int = 115200,
        timeout: float = 0.1,
    ):
        """
        Initialization: finds the device by VID/PID and opens the serial port.

        :param vid: Vendor ID of the device.
        :param pid: Product ID of the device.
        :param baudrate: Baud rate (default is 115200).
        :param timeout: Port timeout (default is 0.1 sec).
        """
        self.vid = vid
        self.pid = pid
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = self.find_and_open_port()

    def find_device_by_vid_pid(self):
        """
        Searches for a device with the given VID and PID.

        :return: The device port name (e.g., 'COM3' or '/dev/ttyUSB0'), or None if not found.
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == self.vid and port.pid == self.pid:
                return port.device
        print("No device found")
        return None

    def find_and_open_port(self):
        """
        Finds the device by VID/PID and opens the serial port.

        :return: A serial.Serial object or None if the device is not found.
        """
        device_port = self.find_device_by_vid_pid()
        if device_port:
            print(f"Device found on port: {device_port}")
            try:
                port = serial.Serial(
                    device_port, baudrate=self.baudrate, timeout=self.timeout
                )
                if port.isOpen():
                    print(f"Port {device_port} opened successfully")
                    return port
            except Exception as e:
                print(f"Failed to open port {device_port}: {e}")
        print("No device found")
        return None

    def calibrate(
        self,
        hand: str,
        DoF_quantity: str,
        DoF: str,
        CW_CCW_mode: str,
        first_transfer: bool = True,
    ):
        """
        Calibration function, sends command via USB.

        :param hand: 'l' (left hand) or 'r' (right hand), or 'left'/'right'
        :param DoF_quantity: '6', '7', or '8'
        :param DoF: a value between '1' and '8'
        :param CW_CCW_mode: '0' or '1', or 'cw' (if 1) / 'ccw' (if 0)
        :param first_transfer: True for the first transfer (delay 1.2 sec), False for subsequent ones (delay 0.3 sec)
        """
        if self.port is None:
            print("Port is not open.")
            return

        # Convert hand parameter
        if hand in ("left", "l"):
            hand = "l"
        elif hand in ("right", "r"):
            hand = "r"
        else:
            raise ValueError("hand must be 'left' or 'right'")

        # Check DoF_quantity
        if DoF_quantity not in ("6", "7", "8"):
            raise ValueError("DoF_quantity must be '6', '7' or '8'")

        # Check DoF
        if DoF not in ("1", "2", "3", "4", "5", "6", "7", "8"):
            raise ValueError("DoF must be between '1' and '8'")

        # Convert CW_CCW_mode parameter
        if CW_CCW_mode in ("cw"):  # If this is gripper "8" this is open state
            CW_CCW_mode = "0"
        elif CW_CCW_mode in ("ccw"):  # If this is gripper "8" this is close state
            CW_CCW_mode = "1"
        else:
            raise ValueError("CW_CCW_mode must be 'cw' or 'ccw'")

        command = f"c{hand}{DoF_quantity}{DoF}{CW_CCW_mode}"
        self.port.write(bytearray(command, "utf-8"))

        time.sleep(1.2 if first_transfer else 0.3)
        data = self.port.read(4)
        error = np.frombuffer(data[:4], dtype=np.uint32)[0]
        if error == 0:
            print("Successful recording!")
        else:
            print(f"Error: {error}")

    def read_data(self, read_force_sensor=True):
        """
        Sends a command to read sensor data from the device and processes the received data.

        The function sends a command to the microcontroller depending on the 'read_force_sensor' flag:
        - If True, it sends "r1" to request both sensor positions and force sensor data.
        - If False, it sends "r2" to request only sensor positions without force sensor data.

        After a short transmission delay, it reads the expected number of bytes and parses them as follows:

        If read_force_sensor is True:
        - DoF_positions (np.array): An array of int16 values (32 bytes) representing positions of the degrees of freedom.
        - force_sensor_4_value (np.array): An array of uint16 values (8 bytes) representing the force sensor readings.
        - sense_flag (uint8): A flag (1 byte) indicating sensitivity operating modes.
        - start_dataset_collection_flag (uint8): A flag (1 byte) indicating whether to start dataset collection.

        If read_force_sensor is False:
        - DoF_positions (np.array): An array of int16 values (32 bytes) representing positions of the degrees of freedom.
        - sense_flag (uint8): A flag (1 byte) indicating sensitivity operating modes.
        - start_dataset_collection_flag (uint8): A flag (1 byte) indicating whether to start dataset collection.

        :param read_force_sensor: Boolean flag to determine if force sensor data should be read (default True).
        :return: A tuple containing the parsed data.
                If read_force_sensor is True, returns
                (DoF_positions, force_sensor_4_value, sense_flag, start_dataset_collection_flag).
                If False, returns (DoF_positions, sense_flag, start_dataset_collection_flag).
        """
        if self.port is None:
            print("Port is not open.")
            return None

        try:
            # Choose command and expected data length based on read_force_sensor flag
            if read_force_sensor:
                command = bytearray(b"r1")
                expected_length = 42
            else:
                command = bytearray(b"r2")
                expected_length = 34

            self.port.write(command)
            time.sleep(0.01)  # 10 ms delay for transmission

            data = self.port.read(expected_length)
            if len(data) < expected_length:
                raise ValueError(
                    f"Received incomplete data: {len(data)} bytes, expected {expected_length} bytes"
                )

            # Parse common data: DoF_positions from the first 32 bytes
            DoF_positions = np.frombuffer(data[:32], dtype=np.int16)

            if read_force_sensor:
                # Parse force sensor data (8 bytes), then sense_flag (1 byte) and start_dataset_collection_flag (1 byte)
                force_sensor_4_value = np.frombuffer(data[32:40], dtype=np.uint16)
                sense_flag = np.frombuffer(data[40:41], dtype=np.uint8)[0]
                start_dataset_collection_flag = np.frombuffer(
                    data[41:], dtype=np.uint8
                )[0]
                return (
                    DoF_positions,
                    force_sensor_4_value,
                    sense_flag,
                    start_dataset_collection_flag,
                )
            else:
                # Parse sense_flag (1 byte) and start_dataset_collection_flag (1 byte) immediately after DoF_positions
                sense_flag = np.frombuffer(data[32:33], dtype=np.uint8)[0]
                start_dataset_collection_flag = np.frombuffer(
                    data[33:34], dtype=np.uint8
                )[0]
                return (DoF_positions, sense_flag, start_dataset_collection_flag)

        except (ValueError, IndexError, OSError) as e:
            print(f"Error reading sensor data: {e}")
            if read_force_sensor:
                return None

            #     return (
            #         np.full(
            #             16, np.nan, dtype=np.float32
            #         ),  # DoF_positions: 16 elements (32 bytes)
            #         np.full(
            #             4, np.nan, dtype=np.float32
            #         ),  # force sensor data: 4 elements (8 bytes)
            #         np.nan,  # sense_flag
            #         np.nan,  # start_dataset_collection_flag
            #     )
            # else:
            #     return (
            #         np.full(
            #             16, np.nan, dtype=np.float32
            #         ),  # DoF_positions: 16 elements (32 bytes)
            #         np.nan,  # sense_flag
            #         np.nan,  # start_dataset_collection_flag
            #     )

    def read_pose_rad(self, dof_count=7, read_force_sensor=True):
        """
        Reads sensor data and converts the DoF positions to radians,
        except for the gripper values at indices 7 and 15, which remain unchanged.

        Angle of rotation of joints = 300 degrees = 5.23598775598 rad
        1 tick = 5.23598775598 / 4096 = 0.0012783173232373046875 rad

        Conversion factor: 1 tick = 0.0012783173232373046875 rad

        :param dof_count: Number of degrees of freedom (6, 7, or 8).
        :param read_force_sensor: Boolean flag indicating whether to read force sensor data.
        :return: A tuple containing:
                - DoF positions for the left arm in radians.
                - DoF positions for the right arm in radians.
                - Force sensor data (if read_force_sensor is True).
                - sense_flag (uint8).
                - start_dataset_collection_flag (uint8).
        """
        # Get sensor data using the read_data method
        data = self.read_data(read_force_sensor=read_force_sensor)
        if data is None:
            return None

        # Extract DoF positions from the received data
        DoF_positions = data[0]
        left_gripper_position = data[0][7].copy()
        right_gripper_position = data[0][15].copy()
        conversion_factor = 0.0012783173232373046875

        # Convert positions to radians
        DoF_positions_rad = DoF_positions.astype(np.float64) * conversion_factor

        # Remove specific indices based on dof quantity
        if dof_count != 8:
            DoF_positions_rad = np.delete(DoF_positions_rad, [6, 7, 14, 15])

        # Split into left and right arm
        DoF_positions_rad_left_arm = DoF_positions_rad[:6]
        DoF_positions_rad_right_arm = DoF_positions_rad[6:]

        # Return separated DoF positions and Gripper positions along with the rest of the data
        if dof_count != 6:
            return (
                (DoF_positions_rad_left_arm, DoF_positions_rad_right_arm)
                + (left_gripper_position, right_gripper_position)
                + data[1:]
            )
        else:
            return (DoF_positions_rad_left_arm, DoF_positions_rad_right_arm) + data[1:]

    def erase_flash(self):
        """
        Sends the erase flash command to the microcontroller.
        """
        if self.port is None:
            print("Port is not open.")
            return
        self.port.write(bytearray(b"e"))
        print("Erase flash command sent.")

    def read_cw_ccw(self):
        """
        Requests CW and CCW values from the microcontroller.
        """
        if self.port is None:
            print("Port is not open.")
            return
        self.port.write(bytearray(b"x"))
        time.sleep(0.01)
        data = self.port.read(64)
        CCW = np.frombuffer(data[:32], dtype=np.uint16)
        CW = np.frombuffer(data[32:], dtype=np.uint16)
        print(f"CCW: {CCW}")
        print(f"CW: {CW}")
