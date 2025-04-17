# ROS2 Package for Autopilot of CavePI AUV

## Compatible Platform

The package is tested and verified on Raspberry Pi 5 with ROS2 Humble.

## Connections
Connect the three USB ports of Raspberry Pi 5 to the ethernet port of Jetson Nano, micro USB port of the Pixhawk, and a USB to UART device to connect with the Ping2 sonar.

## Usage

1. Clone the repo and place the `auv_rpi` folder inside `~/ros2_ws/src/` directory of your Raspberry Pi 5.
2. Build the ROS package.
   ```sh
   cd ~/ros2_ws/
   colcon build --packages-select auv_rpi
   ```
3. Execute the launch file.
   ```sh
   ros2 launch auv_rpi cavepi_auv.launch.py
   ```
   If a caveline is detected from the downward camera, the AUV will start following the caveline.
