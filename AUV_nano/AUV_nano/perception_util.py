# =======================================================
# Author : Alakrit Gupta, Adnan Abdullah
# Email: gupta.alankrit@ufl.edu, adnanabdullah@ufl.edu
# =======================================================
 
import cv2
import time
import numpy as np
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
from PIL import Image
import os

EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
TRT_LOGGER = trt.Logger(trt.Logger.INFO)

batch = 1
host_inputs  = []
cuda_inputs  = []
host_outputs = []
cuda_outputs = []
bindings = []
save_mode = False

# engine_file = "weights/deeplabmbnet_3.engine"
# video_file = "data/tank_rope_skipped.avi"
# output_folder = "data/results/output_frames_mbnet"
# overlay_folder = "data/results/overlay_frames_mbnet"
# os.makedirs(output_folder, exist_ok=True)
# os.makedirs(overlay_folder, exist_ok=True)

_w, _h = 256, 256

def preprocess(frame):
    global _w, _h
    frame = cv2.resize(frame, (_w, _h))
    input_image = (2.0 / 255.0) * frame.transpose((2, 0, 1)) - 1.0
    return input_image

def postprocess(output, original_frame_shape):
    global _w, _h
    output = np.reshape(output, (2, _w, _h))
    segmentation_map = np.argmax(output, axis=0)  # Shape becomes (256, 256)
    segmentation_image = (segmentation_map * 255).astype('uint8')
    segmentation_image = cv2.resize(segmentation_image, (original_frame_shape[1], original_frame_shape[0]))
    segmentation_image = cv2.merge((segmentation_image,segmentation_image,segmentation_image))
    return segmentation_image

def overlay_segmentation(input_frame, segmentation_map, alpha=0.75):
    overlay = cv2.addWeighted(input_frame, alpha, segmentation_map, 1-alpha, 0)
    return overlay

def sort_contours(cnts, method="right-to-left"):
	# initialize the reverse flag and sort index
	reverse = False
	i = 0
	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True
	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1
	# construct the list of bounding boxes and sort them from top to
	# bottom
	boundingBoxes = [cv2.boundingRect(c) for c in cnts]
	(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
		key=lambda b:b[1][i], reverse=reverse))
	# return the list of sorted contours and bounding boxes
	return (cnts, boundingBoxes)


def find_line_points(segmentation_map):
    line_points = []
    # Apply Canny edge detection
    edges = cv2.Canny(segmentation_map, 50, 150)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Sort the contours from right to left (robot's direction)
    sorted_contours, _ = sort_contours(contours, method='right-to-left')

    # Fit a line to the contours (using polyfit to fit a line that goes through the contour)
    for contour in sorted_contours:
        if len(contour) >= 5:  # Fit a line only if the contour has enough points
            # Extract x and y coordinates of the contour points
            points = contour.reshape(-1, 2)  # Convert to 2D array of points (x, y)

            # Use np.polyfit to fit a line (degree 1 = linear fit)
            # The polyfit function returns the slope and intercept of the line
            [slope, intercept] = np.polyfit(points[:, 0], points[:, 1], 1)

            # Compute the line's start and end points for visualization
            # We can extend the line by using the image's x-range
            x1, x2 = min(points[:, 0]), max(points[:, 0])
            y1, y2 = slope * x1 + intercept, slope * x2 + intercept

            # Draw the fitted line on the original image (in color for visibility)
            cv2.line(segmentation_map, (int(x1), int(y1)), (int(x2), int(y2)), (100, 255, 100), 5)
            # print(f"line slope: {slope}, intercept: {intercept}")
            line_points.append([x1, y1, x2, y2])
    return line_points, segmentation_map


def InferenceOnFrame(bindings, host_inputs, cuda_inputs, host_outputs, cuda_outputs, stream, context, frame, frame_idx=0):
    global _w, _h

    try:
        original_frame = frame.copy()
        input_image = preprocess(frame)

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

        line_ponts, line_map = find_line_points(segmentation_map)

        # Overlay
        line_overlayed_map = overlay_segmentation(original_frame, line_map)

        # if save_mode:
        #     # Save segmentation output
        #     output_path = os.path.join(output_folder, f"frame_{frame_idx:04d}.png")
        #     cv2.imwrite(output_path, segmentation_map)

        #     # Save overlay
        #     overlay_path = os.path.join(overlay_folder, f"frame_{frame_idx:04d}.png")
        #     cv2.imwrite(overlay_path, line_overlayed_map)

        return line_ponts, line_overlayed_map
    except:
	    return None, frame
    
def PrepareEngine(engine_file):
    with open(engine_file, 'rb') as f:
        serialized_engine = f.read()

    runtime = trt.Runtime(TRT_LOGGER)
    engine = runtime.deserialize_cuda_engine(serialized_engine)

    # Create buffer
    for binding in engine:
        size = trt.volume(engine.get_tensor_shape(binding)) * batch
        host_mem = cuda.pagelocked_empty(shape=[size], dtype=np.float32)
        cuda_mem = cuda.mem_alloc(host_mem.nbytes)

        bindings.append(int(cuda_mem))
        if engine.get_tensor_mode(binding) == trt.TensorIOMode.INPUT:
            host_inputs.append(host_mem)
            cuda_inputs.append(cuda_mem)
        else:
            host_outputs.append(host_mem)
            cuda_outputs.append(cuda_mem)
    
    stream = cuda.Stream()
    context = engine.create_execution_context()

    return engine, bindings, host_inputs, cuda_inputs, host_outputs, cuda_outputs, stream, context

# if __name__ == "__main__":
#     engine = PrepareEngine(engine_file)
#     InferenceOnFrame(engine, video_file, output_folder, overlay_folder)
#     engine = []
