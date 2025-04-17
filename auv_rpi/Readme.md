# ROS2 Package for Autopilot of CavePI AUV

## Compatible Platform

The package is tested and verified on Raspberry Pi 5 with ROS2 Humble.

## Connections
1. Connect Raspberry Pi 5's one of the USB ports to the ethernet port of Jetson Nano.
2. Coneect one of the other USB ports of Raspberry Pi 5 to the micro USB port of the Pixhawk.
3. Connect 

## Usage

1. Clone the repo and place the `auv_rpi` folder inside `~/ros2_ws/src/` directory of your  device.
2. Download the model weights from here: [Dropbox link](https://www.dropbox.com/scl/fo/6oin10fofx2k8ffhxluia/AJO9DvS03urmhyW1etIEWww?rlkey=bu4xx6g4re4qdunjx313njqqo&st=e0ep0fvo&dl=0)
   
   You will find `.onnx` files as well as `.engine` files. If your platform matches the configuration described above, you can directly use the `.engine` files. For different platforms such as Jetson Orin, download the `.onnx` file in your Jetson device and convert it to `.engine` format:
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=<onnx_filename> --saveEngine=<engine_filename>
   ```
3. Place the engine file inside `~/ros2_ws/src/AUV_nano/weights/` directory
4. Set the `engine_filename` in `detector.py`. The default is `mobilenet.engine`.
5. Connect the two cameras to USB ports of your Jetson device. Check their port number using `lsusb` command.
6. Update the port numbers: `FRONT_CAMERA_INDEX` and `DOWN_CAMERA_INDEX` in `config/config.yaml`.  

7. Build the ROS package.
   ```sh
   cd ~/ros2_ws/
   colcon build --packages-select AUV_nano
   ```
8. Execute the launch file. It will run the detector and planner node.
   ```sh
   ros2 launch AUV_nano detector_and_planner.launch.py
   ```
   If a line is detected from the downward camera, the planner will send control commands to Raspberry Pi. Please refer to the `AUV_rpi` folder to setup the Raspberry Pi codes.
