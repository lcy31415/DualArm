import os

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BASE_DIR = os.path.dirname(__file__)

CAMERA_INDEX = 1

STOP_POSITION = {"x": 80.0, "y": 220.0, "z": 85.0, "r": 0.0}
SAFE_POSITION = {"x": 80.0, "y": 220.0, "z": 85.0, "r": 0.0}
PLACE_POSITIONS = {
    "red": {"x": 125.0, "y": -200.0, "z": 0.0, "r": 0.0},
    "green": {"x": 50.0, "y": -200.0, "z": 0.0, "r": 0.0},
    "blue": {"x": -15.0, "y": -200.0, "z": 0.0, "r": 0.0},
}

Z_WORKPLANE = 0.0
APPROACH_Z = 30.0
PICK_Z = -18.0
PLACE_Z = -18.0
FLIP_Y = True

CAMERA_PARAMS_PATH = os.path.join(ROOT_DIR, "camera_params.npz")
HANDEYE_RESULT_PATH = os.path.join(ROOT_DIR, "handeye_result.npz")
COORDINATES_FILE = os.path.join(BASE_DIR, "data", "coordinates.json")
COMMAND_FILE = os.path.join(BASE_DIR, "data", "command.json")

USE_GRIPPER = True
USE_SUCTION = False
GRAB_R = 0.0
