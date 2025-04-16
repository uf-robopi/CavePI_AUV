# ROS2 Package for Perception and Control of CavePI AUV

## Compatible Platform
Jetson Nano 2GB

Jetpack 6

Python 3.10

Cuda xx

Tensor RT xx

## Usage:


1. Download and place this folder inside your `ros2_ws/src` directory.
2. Download the model weights from here: [Dropbox link](www.somelink.com)
   
   You will find .onnx files as well as .engine files. If your platform matches the configuration described above, you can directly use the .engine files. Otherwise, download the .onnx file in your Jetson device and convert it to .engine format:
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
   ```
4. Build the ROS package.
   ```sh
   colcon build --packages-select <package_name>
   ```
5. Run the caveline detector node.
   ```sh
   ros2 run detector caveline_detection_main.py
   ```


