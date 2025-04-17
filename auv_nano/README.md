# ROS2 Package for Perception of CavePI AUV

## Compatible Platform

The package is tested and verified on Jetson Orin Nano. It also runs on Jetson Nano 2GB with minimal modification in the code. Please refer to the `perception_utils.py` and comment/uncomment the codes where Jetson Nano and Jetson Orin Nano are mentioned.

Other specs (Platform dependent):

|            | Jetson Nano 2GB   | Jetson Orin Nano   |
| ---------  | ----------------  | ------------------ |
| Ubuntu     |      18.04        |    22.04           |
| ROS2       |      Foxy         |    Humble          |
| Jetpack    |    4.6.3          |      6.0           |
| Python     |      3.6.9        |      3.10.6        |
| Cuda       |      10.2         |      12.1          |
| CuDNN      |      8.2.1        |      9.1           |
| TensorRT   |      8.2.1        |      8.6           |



## Usage

1. Clone the repo and place the `auv_nano` folder inside `~/ros2_ws/src/` directory of your jetson device.
2. Download the model weights from here: [Dropbox link](https://www.dropbox.com/scl/fo/6oin10fofx2k8ffhxluia/AJO9DvS03urmhyW1etIEWww?rlkey=bu4xx6g4re4qdunjx313njqqo&st=e0ep0fvo&dl=0)
   
   You will find `.onnx` files as well as `.engine` files. If your platform matches the Jetson Nano 2GB configuration described above, you can directly use the `.engine` files. For different platforms such as Jetson Orin, download the `.onnx` file in your Jetson device and convert it to `.engine` format:
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
   ```
3. Create the `~/ros2_ws/src/auv_nano/weights/` directory and place the engine file there.
   ```sh
   mkdir -p ~/ros2_ws/src/auv_nano/weights/
   mv <path_to_your_engine_file> ~/ros2_ws/src/auv_nano/weights/
   ```
5. Set the `engine_filename` in `detector.py`. The default is `mobilenet.engine`.
6. Connect the two cameras to USB ports of your Jetson device. Check their port number using `lsusb` command.
7. Update the port numbers: `FRONT_CAMERA_INDEX` and `DOWN_CAMERA_INDEX` in `config/config.yaml`.  

8. Build the ROS package.
   ```sh
   cd ~/ros2_ws/
   colcon build --packages-select auv_nano
   ```
9. Execute the launch file. It will run the detector and planner node.
   ```sh
   ros2 launch auv_nano detector_and_planner.launch.py
   ```
   If a line is detected from the downward camera, the planner will send control commands to Raspberry Pi. Please refer to the `auv_rpi` folder to setup the Raspberry Pi codes.


