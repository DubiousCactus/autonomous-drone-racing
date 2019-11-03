# Exploiting Fast and Accurate Semi-Synthetic Image Generation for Efficient CNN Training

This repository contains the source code for the paper "Exploiting Fast and
Accurate Semi-Synthetic Image Generation for Efficient CNN Training". 

[![Watch the video](https://img.youtube.com/vi/U4LolcYm44w/maxresdefault.jpg)](https://youtu.be/U4LolcYm44w)

## ROS nodes

The nodes that can be ran onboard are located in the following packages:

- `controllers`: which runs the camera driver, the Vicon bridge and the safety cage
- `img_center_alignment`: which is the state machine and PID controller
- `perception`: which runs the two CNNs (bounding box detection + distance estimation)

The three launch files are `controllers control_inteldrone.launch`,
`perception detector.launch`, and `img_center_alignment controller.launch`.
It is required to launch them in that specific order. Note that it is also
recommended to launch the perception node on a stationary computer, since the
inference is quite demanding.

## Dataset creation

Use the **hybrid-dataset-factory** project, which can be found at
[https://github.com/M4gicT0/hybrid-dataset-factory](https://github.com/M4gicT0/hybrid-dataset-factory).
The mesh files used to generate the dataset in this paper can be found in the
**meshes/** folder.

## Gate detection

The complete CNN model and code is provided in the **perception** package.
However, if one desires to study the code in detail, the repository is
available at
[https://github.com/M4gicT0/mobilenet_v2_ssdlite_keras](https://github.com/M4gicT0/mobilenet_v2_ssdlite_keras)
in the `custom_dataset` branch.


## Gate distance estimation

For the distance estimation, only the saved model is given in the
**perception** package. The source code is available at
[https://github.com/M4gicT0/GatePoseEstimator](https://github.com/M4gicT0/GatePoseEstimator).
It is still a work-in-progress but feel free to contribute.
