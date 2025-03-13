import pyrealsense2 as rs
import numpy as np
import time


class RealSenseCamera:
    """
    A class to capture color frames from an Intel RealSense camera.

    This class initializes the RealSense pipeline, configures the camera to
    stream color data, and provides a method to retrieve frames as NumPy arrays.
    The capture frequency determines the minimum time interval between successive captures.
    """

    def __init__(self, capture_frequency=30, width=1280, height=720):
        """
        Initialize the RealSenseCamera.

        Parameters:
            capture_frequency (int): Number of frames per second to capture.
                                     This value is used to compute the time interval between captures.
        """
        # # Calculate the minimum time (in seconds) between captures based on the given frequency.
        # self.capture_interval = 1 / capture_frequency

        # Create a pipeline object to manage the stream of data from the camera.
        self.pipeline = rs.pipeline()

        # Create a configuration for the pipeline. This specifies which streams to enable.
        self.config = rs.config()
        # Enable the color stream from the camera with the specified resolution and frame rate.
        self.config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, capture_frequency
        )

        # Start the RealSense pipeline using the configuration.
        self.pipeline.start(self.config)

        # # Initialize the time of the last captured frame to zero.
        # self.last_capture_time = 0

    def get_frame(self):
        """
        Capture and return a color frame as a NumPy array if the capture interval has elapsed.

        Returns:
            frame_array (numpy.ndarray): The captured color frame as a NumPy array.
                                         Returns None if the capture interval has not passed or if no frame is captured.
        """
        # # Get the current time.
        # current_time = time.time()

        # # Check if the required time interval has passed since the last capture.
        # if current_time - self.last_capture_time < self.capture_interval:
        #     # If not enough time has passed, do not capture a new frame.
        #     return None

        # Wait for a new set of frames from the camera.
        frames = self.pipeline.wait_for_frames()

        # Extract the color frame from the frameset.
        color_frame = frames.get_color_frame()
        if not color_frame:
            # If the color frame is unavailable, return None.
            return None

        # Convert the color frame data to a NumPy array.
        frame_array = np.asanyarray(color_frame.get_data())

        # # Update the last capture time to the current time.
        # self.last_capture_time = current_time

        # Return the captured frame as a NumPy array.
        return frame_array

    def release(self):
        """
        Stop the RealSense pipeline and release camera resources.

        Call this method when you no longer need to capture frames.
        """
        self.pipeline.stop()
