from pathlib import Path
import pandas as pd
import numpy as np
import cv2


class DataCollector:
    """
    The DataCollector class is responsible for managing the dataset structure, which includes:

    - A CSV file for logging data samples.
    - An 'img' folder to store images for each data sample.
    - A unique folder for each trajectory, named as `trajectory_<number>`, where images and CSV data for that trajectory will be stored.

    Each sample logged in the CSV file contains:
    - Left Arm joints: a 6-element array representing the joint positions of the left arm.
    - Right Arm joints: a 6-element array representing the joint positions of the right arm.
    - Left Arm gripper: the position of the left gripper.
    - Right Arm gripper: the position of the right gripper.
    - Path to image: the path to the saved image file, stored as a string.
    """

    def __init__(self, dataset_dir="dataset"):
        # Set up dataset directory and create it if it doesn't exist.
        self.dataset_dir = Path(dataset_dir)
        self.dataset_dir.mkdir(exist_ok=True)

        # Define column names for CSV
        self.columns = [
            # Left arm
            "Left Arm joint 1",
            "Left Arm joint 2",
            "Left Arm joint 3",
            "Left Arm joint 4",
            "Left Arm joint 5",
            "Left Arm joint 6",
            "Left Arm gripper",
            # Right arm
            "Right Arm joint 1",
            "Right Arm joint 2",
            "Right Arm joint 3",
            "Right Arm joint 4",
            "Right Arm joint 5",
            "Right Arm joint 6",
            "Right Arm gripper",
            # Path to image
            "Path to frame",
        ]

    def new_trajectory(self):
        """
        Creates a new trajectory folder with a unique number, a CSV file, and an img folder.
        """
        # Find the next available trajectory number (e.g., trajectory_1, trajectory_2, ...)
        trajectory_number = 1
        while (self.dataset_dir / f"trajectory_{trajectory_number}").exists():
            trajectory_number += 1

        # Create the new trajectory folder
        trajectory_folder = self.dataset_dir / f"trajectory_{trajectory_number}"
        trajectory_folder.mkdir(exist_ok=True)

        # Create the img folder inside the trajectory folder
        img_folder_path = trajectory_folder / "img"
        img_folder_path.mkdir(exist_ok=True)

        # Create a CSV file inside the trajectory folder
        csv_file_path = trajectory_folder / "data.csv"

        # If the CSV file doesn't exist, create it with headers
        if not csv_file_path.exists():
            df = pd.DataFrame(columns=self.columns)
            df.to_csv(csv_file_path, index=False)

        # Return the paths to the CSV and img folder for later use
        return csv_file_path, img_folder_path

    def add_sample(
        self,
        left_arm_new_position,
        right_arm_new_position,
        left_gripper_position,
        right_gripper_position,
        frame,
        csv_file_path,
        img_folder_path,
    ):
        """
        Adds a new data sample.

        Parameters:
            left_arm_new_position (numpy.ndarray): 6-element array for the left arm joints.
            right_arm_new_position (numpy.ndarray): 6-element array for the right arm joints.
            left_gripper_position (int): Left gripper position.
            right_gripper_position (int): Right gripper position.
            frame (numpy.ndarray): Image data.
            csv_file_path (Path): Path to the CSV file where data will be logged.
            img_folder_path (Path): Path to the folder where images will be stored.
        """
        # Create an image filename based on the current timestamp
        img_filename = f"{pd.Timestamp.now().strftime('%Y%m%d_%H%M%S%f')}.png"
        img_path = img_folder_path / img_filename

        # Save the image
        cv2.imwrite(str(img_path), frame)

        # Prepare a list with data to be logged
        data = [
            *left_arm_new_position.tolist(),
            left_gripper_position.item(),
            *right_arm_new_position.tolist(),
            right_gripper_position.item(),
            str(img_path),
        ]

        # Create a DataFrame to add a row to the CSV
        df = pd.DataFrame([data], columns=self.columns)
        df.to_csv(csv_file_path, mode="a", header=False, index=False)
