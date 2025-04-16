# =======================================================
# Author : Alakrit Gupta, Adnan Abdullah
# Email: gupta.alankrit@ufl.edu, adnanabdullah@ufl.edu
# =======================================================

import cv2
import time
import socket
import json
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory


#######################################
# Configuration
#######################################

# Load configuration
package_share = get_package_share_directory('detector')
config_file = os.path.join(package_share, 'config', 'config.yaml')
with open(config_file, 'r') as f:
    config = yaml.safe_load(f)


FRONT_CAMERA_INDEX = config['FRONT_CAMERA_INDEX']
DOWN_CAMERA_INDEX = config['DOWN_CAMERA_INDEX']


# Camera attempt parameters
MAX_ATTEMPTS = config['MAX_ATTEMPTS']
REINIT_ATTEMPTS = config['REINIT_ATTEMPTS']


# Frame properties (width, height, fps)
FRAME_WIDTH = config['FRAME_WIDTH']
FRAME_HEIGHT = config['FRAME_HEIGHT']
FRAME_FPS = config['FRAME_FPS']


# Publish rate (Hz) - how often we process frames and send data
PUBLISH_RATE = config['PUBLISH_RATE']  # 5 Hz
SLEEP_TIME = 1.0 / PUBLISH_RATE


# UDP settings
UDP_IP = config['UDP_IP']
UDP_PORT = config['UDP_PORT'] 



#######################################
# Helper functions
#######################################
def initialize_camera(index, camera_name):
    """
    Attempt to open a camera multiple times. Returns an open cv2.VideoCapture
    if successful, otherwise a closed one (camera.isOpened()=False).
    """
    print(f"[INFO] Initializing {camera_name} on index {index} ...")
    camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, FRAME_FPS)

    attempt = 0
    while not camera.isOpened() and attempt < MAX_ATTEMPTS:
        print(f"[WARN] Cannot open {camera_name} at index {index}. Retrying {attempt+1}/{MAX_ATTEMPTS}...")
        time.sleep(1)
        camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, FRAME_FPS)
        attempt += 1

    if not camera.isOpened():
        print(f"[ERROR] Failed to open {camera_name} after {MAX_ATTEMPTS} attempts.")
    else:
        print(f"[INFO] {camera_name.capitalize()} opened successfully.")

    return camera

def reinitialize_camera(index, camera_name):
    """
    Re-initialize camera if it fails mid-operation.
    """
    print(f"[INFO] Reinitializing {camera_name} ...")
    camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, FRAME_FPS)

    attempt = 0
    while not camera.isOpened() and attempt < REINIT_ATTEMPTS:
        print(f"[WARN] Failed to reconnect {camera_name}. Retrying {attempt+1}/{REINIT_ATTEMPTS}...")
        time.sleep(1)
        camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, FRAME_FPS)
        attempt += 1

    if not camera.isOpened():
        print(f"[ERROR] Failed to reconnect {camera_name} after {REINIT_ATTEMPTS} attempts.")

    return camera

def send_udp_data(sock, data_type, data, ip, port):
    """
    Send a JSON-encoded dictionary over UDP.
    """
    try:
        message = {
            'type': data_type,
            'data': data
        }
        serialized_message = json.dumps(message).encode('utf-8')
        sock.sendto(serialized_message, (ip, port))
        # Debug printing (optional):
        # print(f"[DEBUG] Sent {data_type} data: {message}")
    except Exception as e:
        print(f"[ERROR] Failed to send {data_type} data: {e}")