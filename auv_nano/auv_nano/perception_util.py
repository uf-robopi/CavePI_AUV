# =======================================================
# Author : Alakrit Gupta, Adnan Abdullah
# Email: gupta.alankrit@ufl.edu, adnanabdullah@ufl.edu
# =======================================================

"""
This script contains helper functions for processing images and performing inference using TensorRT.
"""

import cv2
import time
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # This automatically initializes CUDA driver

# Initialize CUDA variables
EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
TRT_LOGGER = trt.Logger(trt.Logger.INFO)

batch = 1
host_inputs  = []
cuda_inputs  = []
host_outputs = []
cuda_outputs = []
bindings = []
save_mode = False

# Default image size
_w, _h = 256, 256


#######################################
# Helper functions
#######################################

def preprocess(frame):
    """
    Resize and normalize input image frame for inference.
    Args:
        frame (numpy.ndarray): Input image frame.
    Returns:
        numpy.ndarray: Preprocessed image ready for inference.
    """
    global _w, _h
    frame = cv2.resize(frame, (_w, _h))
    input_image = (2.0 / 255.0) * frame.transpose((2, 0, 1)) - 1.0
    return input_image

def postprocess(output, original_frame_shape):
    """
    Postprocess the output of the model to create a segmentation map.
    Args:
        output (numpy.ndarray): Output from the model.
        original_frame_shape (tuple): Shape of the original frame.
    Returns:
        numpy.ndarray: Segmentation map image.
    """
    global _w, _h
    output = np.reshape(output, (2, _w, _h))
    segmentation_map = np.argmax(output, axis=0)  # Shape becomes (256, 256)
    segmentation_image = (segmentation_map * 255).astype('uint8')
    segmentation_image = cv2.resize(segmentation_image, (original_frame_shape[1], original_frame_shape[0]))
    segmentation_image = cv2.merge((segmentation_image,segmentation_image,segmentation_image))
    return segmentation_image

def overlay_segmentation(input_frame, segmentation_map, alpha=0.75):
    """
    Overlay the segmentation map on the input frame for visualization.
    Args:
        input_frame (numpy.ndarray): Original input frame.
        segmentation_map (numpy.ndarray): Segmentation map to overlay.
        alpha (float): Transparency factor for overlay.
    Returns:
        numpy.ndarray: Overlayed image.
    """
    overlay = cv2.addWeighted(input_frame, alpha, segmentation_map, 1-alpha, 0)
    return overlay

def sort_contours(cnts, method="right-to-left"):
    """
    Sort contours on a frame according to their pixel locations
    Args:
        cnts (list): List of contours to sort.
        method (str): Sorting method - left-to-right, right-to-left, top-to-bottom, or bottom-to-top.
    Returns:
        tuple: Sorted contours and their corresponding bounding boxes.
    """
	# initialize the reverse flag and sort index
    reverse = False
    i = 0
    # if reverse is necessary
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True

    # handle if sorting against the y-coordinate rather than
    # the x-coordinate of the bounding box
    if method == "top-to-bottom" or method == "bottom-to-top":
        i = 1

    # construct the list of bounding boxes and sort them from top to bottom
    boundingBoxes = [cv2.boundingRect(c) for c in cnts]
    (cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
        key=lambda b:b[1][i], reverse=reverse))
    
    # return the list of sorted contours and bounding boxes
    return (cnts, boundingBoxes)


def find_line_points(segmentation_map):
    """
    Find line points in the segmentation map using Canny edge detection and contour fitting.
    Args:
        segmentation_map (numpy.ndarray): Segmentation map to process.
    Returns:
        list: List of line points (start and end coordinates).
        numpy.ndarray: Segmentation map with lines drawn.
    """
    line_points = []

    # Apply Canny edge detection on binary segmentation map
    edges = cv2.Canny(segmentation_map, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Sort the contours from right to left (robot's direction)
    sorted_contours, _ = sort_contours(contours, method='right-to-left')

    # Fit a line to the contours (using polyfit to fit a line that goes through the contour)
    for contour in sorted_contours:
        if len(contour) >= 5:  # Fit a line only if the contour has enough points
            # Extract x and y coordinates of the contour points
            points = contour.reshape(-1, 2) 

            # Fit a line (degree 1 = linear fit)
            [slope, intercept] = np.polyfit(points[:, 0], points[:, 1], 1)

            # Compute the line's start and end points for visualization
            x1, x2 = min(points[:, 0]), max(points[:, 0])
            y1, y2 = slope * x1 + intercept, slope * x2 + intercept

            # Draw the fitted line on the original image (color for visibility)
            cv2.line(segmentation_map, (int(x1), int(y1)), (int(x2), int(y2)), (100, 255, 100), 5)
            # print(f"line slope: {slope}, intercept: {intercept}")
            line_points.append([x1, y1, x2, y2])

    return line_points, segmentation_map


def InferenceOnFrame(bindings, host_inputs, cuda_inputs, host_outputs, cuda_outputs, stream, context, frame, frame_idx=0):
    """
    Perform inference on a single frame using the TensorRT engine.
    Args:
        bindings (list): List of bindings for the TensorRT engine.
        host_inputs (list): List of host input buffers.
        cuda_inputs (list): List of CUDA input buffers.
        host_outputs (list): List of host output buffers.
        cuda_outputs (list): List of CUDA output buffers.
        stream: CUDA stream for asynchronous execution.
        context: TensorRT execution context.
        frame (numpy.ndarray): Input image frame.
        frame_idx (int): Frame index for debugging purposes.
    Returns:
        list: List of line points (start and end coordinates).
        numpy.ndarray: overlayed image with lines drawn.
    """
    global _w, _h

    try:
        original_frame = frame.copy()

        # Preprocess
        input_image = preprocess(frame)

        # Copy input image to host input buffer
        np.copyto(host_inputs[0], input_image.ravel())

        # Inference
        start_time = time.time()
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        context.execute_v2(bindings)
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        stream.synchronize()
        print(f"[INFO] Inference time {time.time() - start_time:.4f} seconds")

        # Postprocess
        segmentation_map = postprocess(host_outputs[0], original_frame.shape)

        # Find contours and their coordinate points
        line_ponts, line_map = find_line_points(segmentation_map)

        # Overlay the segmentation map on the original frame
        line_overlayed_map = overlay_segmentation(original_frame, line_map)

        return line_ponts, line_overlayed_map
    except:
	    return None, frame
    
def PrepareEngine(engine_file):
    """
    Prepare the TensorRT engine for inference.
    Args:
        engine_file (str): Path to the serialized TensorRT engine file.
    Returns: Prepared engine and buffers for inference."""

    # Read the serialized engine
    with open(engine_file, 'rb') as f:
        serialized_engine = f.read()

    # Create a runtime for the engine
    runtime = trt.Runtime(TRT_LOGGER)
    engine = runtime.deserialize_cuda_engine(serialized_engine)

    # Create input and output buffer
    for binding in engine:
        # size = trt.volume(engine.get_binding_shape(binding)) * batch # For Jetson nano
        size = trt.volume(engine.get_tensor_shape(binding)) * batch # For jetson Orin nano
        host_mem = cuda.pagelocked_empty(shape=[size], dtype=np.float32)
        cuda_mem = cuda.mem_alloc(host_mem.nbytes)

        bindings.append(int(cuda_mem))
        # if engine.binding_is_input(binding): # For Jetson nano
        if engine.get_tensor_mode(binding) == trt.TensorIOMode.INPUT: # For jetson Orin nano
            host_inputs.append(host_mem)
            cuda_inputs.append(cuda_mem)
        else:
            host_outputs.append(host_mem)
            cuda_outputs.append(cuda_mem)
    
    # Create a CUDA stream and execution context
    stream = cuda.Stream()
    context = engine.create_execution_context()

    return engine, bindings, host_inputs, cuda_inputs, host_outputs, cuda_outputs, stream, context
