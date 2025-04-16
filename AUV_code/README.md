# CavePI Perception and Control

## Installations:


1. If you have a .pth model, convert to onnx. Ideally it should be done on the host machine.
   ```sh
    python3 torch2onnx.py (require torch and torchvision)
   ```

2. Convert onnx model to tensorRT engine (we are using this).
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=rope_seg_deeplabresnet101_2.onnx --saveEngine=deeplabresnet101_2.engine
   ```
  Or, convert onnx model to tensorRT trt format (we are not using this).
   ```sh
    /usr/src/tensorrt/bin/trtexec --onnx=rope_seg_deeplabresnet101_2.onnx --saveEngine=deeplabresnet101_2.trt --fp16
   ```
4. Build the ROS package.
   ```sh
   colcon build --packages-select detector
   ```
6. Run the caveline detector node.
   ```sh
   ros2 run detector caveline_detection_main.py
   ```


