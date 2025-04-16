# ROS2 Package for Perception of CavePI AUV

## Compatible Platform

The package is tested and verified on Jetson Nano 2GB. It also runs on Jetson Orin Nano with minimal modification in the perception module. Please get in touch with the contributors for Jetson Orin code.

Other specs:
- ROS2 Humble
- Jetpack 4.6.4
- Python 3.6.9
- Cuda 10.2, CuDNN 8.2.1
- Tensor RT 6.0.1.10

## Usage

1. Clone the repo and place the `AUV_nano` folder inside `ros2_ws/src/` directory of your jetson device.
2. Download the model weights from here: [Dropbox link](https://www.dropbox.com/scl/fo/6oin10fofx2k8ffhxluia/AJO9DvS03urmhyW1etIEWww?rlkey=bu4xx6g4re4qdunjx313njqqo&st=e0ep0fvo&dl=0)
   
   You will find `.onnx` files as well as `.engine` files. If your platform matches the configuration described above, you can directly use the `.engine` files. For different platforms such as Jetson Orin, download the `.onnx` file in your Jetson device and convert it to `.engine` format:
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
   ```
3. Set the `engine_filename` in `detector.py`. The default is `mobilenet.engine`.
4. Connect the two cameras to USB ports of your Jetson device. Check their port number using `lsusb` command.
5. Update the port numbers: `FRONT_CAMERA_INDEX` and `DOWN_CAMERA_INDEX` in `config/config.yaml`.  

7. Build the ROS package.
   ```sh
   cd ~/ros2_ws/
   colcon build --packages-select AUV_nano
   ```
8. Execute the launch file. It will run the detector and planner node.
   ```sh
   ros2 launch AUV_nano detector_and_planner.launch.py
   ```


