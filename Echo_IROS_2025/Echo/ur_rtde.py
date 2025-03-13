from time import sleep
import numpy as np
import rtde_control
import rtde_receive
import robotiq_gripper


class UR3Teleop:

    def __init__(
        self,
        ip,
        base_pose,
        velocity=2.2,
        acceleration=4.0,
        dt=1.0 / 500,  # 2ms
        lookahead_time=0.2,  # d coeff
        gain=200,  # p coeff
        gripper_velocity=255,
        gripper_force=200,
        use_gripper=True,  # New parameter: whether a gripper is present
    ):
        """
        Initializes the connection to the robot using the given IP address and stores the base pose.
        Optionally initializes the gripper if use_gripper is True.

        :param ip: Robot's IP address.
        :param base_pose: Base pose as a list or numpy array of 6 joint angles in radians.
        :param velocity: Movement speed (default is 2.2).
        :param acceleration: Acceleration (default is 4.0).
        :param dt: Update period (default is 1/500).
        :param lookahead_time: Lookahead time (default is 0.2).
        :param gain: Gain factor (default is 200).
        :param gripper_velocity: Gripper movement speed.
        :param gripper_force: Gripper force.
        :param use_gripper: Flag to indicate if a gripper is available.
        """
        self.ip = ip
        self.base_pose = (
            base_pose if isinstance(base_pose, list) else base_pose.tolist()
        )
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.velocity = velocity
        self.acceleration = acceleration
        self.dt = dt
        self.lookahead_time = lookahead_time
        self.gain = gain

        self.gripper_velocity = gripper_velocity
        self.gripper_force = gripper_force
        self.use_gripper = use_gripper

        if self.use_gripper:
            self.gripper = robotiq_gripper.RobotiqGripper()
            self.gripper.connect(ip, 63352)
            try:
                # Attempt to activate the gripper if it is not active
                if not self.gripper.is_active():
                    self.gripper.activate()
                # Check if the gripper is active after the activation attempt
                if not self.gripper.is_active():
                    raise Exception("Gripper activation failed")
            except Exception as e:
                error_message = "Error: could not connect gripper"
                print(error_message)
                raise Exception(error_message) from e
        else:
            self.gripper = None

    def move_to_pose(self, joints_positions, gripper_position=None):
        """
        Moves the robot to the specified joints_positions using the servoJ command.

        :param joints_positions: A list or numpy array of 6 joint angles in radians.
        :param gripper_position: The desired gripper position (if a gripper is available).
        """
        if isinstance(joints_positions, np.ndarray):
            joints_positions = joints_positions.tolist()

        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoJ(
            joints_positions,
            self.velocity,
            self.acceleration,
            self.dt,
            self.lookahead_time,
            self.gain,
        )
        self.rtde_c.waitPeriod(t_start)

        # If a gripper is present and a target position is provided, move the gripper
        if self.use_gripper and gripper_position is not None:
            self.gripper.move(
                gripper_position, self.gripper_velocity, self.gripper_force
            )

    def move_to_base_pose(self):
        """
        Moves the robot to its stored base pose.
        """
        self.move_to_pose(self.base_pose)

    def get_current_pose(self):
        """
        Retrieves the current joint positions of the robot.

        :return: A list of 6 values (joint angles in radians).
        """
        return self.rtde_r.getActualQ()
