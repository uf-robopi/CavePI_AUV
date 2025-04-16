# ROS2 Package for Perception and Control of CavePI AUV

## Compatible Platform
Jetson Nano 2GB

Jetpack 4.6.4

Python 3.6.9

Cuda 10.2, CuDNN 8.2.1

Tensor RT 6.0.1.10

## Usage:

1. Download and place this folder inside your `ros2_ws/src` directory.
2. Download the model weights from here: [Dropbox link](www.somelink.com)
   
   You will find `.onnx` files as well as `.engine` files. If your platform matches the configuration described above, you can directly use the `.engine` files. For different platforms such as Jetson Orin, download the `.onnx` file in your Jetson device and convert it to `.engine` format:
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
   ```
4. Connect the two cameras to USB ports of your Jetson device. Check their port number using
   ```sh
   code
   ```
   Update the port numbers `FRONT_CAMERA_INDEX` and `DOWN_CAMERA_INDEX` in `config/config.yaml`.  
6. Set the `engine_filename` in `detector.py`. Default engine is `deeplabmbnet_3.engine`.
7. Build the ROS package.
   ```sh
   colcon build --packages-select AUV_nano
   ```
8. Execute the launch file. It will run the detector and planner node.
   ```sh
   ros2 launch AUV_nano detector_and_planner.launch.py
   ```


