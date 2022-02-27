# Robotics Toolbox For Serial Manipulators ![SHIELD](https://img.shields.io/badge/Project%20Status%3A-Ongoing-yellow?style=for-the-badge) ![c++](https://camo.githubusercontent.com/6301a47e098ea0b84260920a75b5a71f121c5a0b55965dff8ad80bd60db208c7/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d4325324225324226636f6c6f723d303035393943266c6f676f3d43253242253242266c6f676f436f6c6f723d464646464646266c6162656c3d)![ROBOTICS](https://camo.githubusercontent.com/b8e2732eda54a502cb34a56c1ea83747134ce98754e6c49a3177cd89f411bc97/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d526f626f742b4672616d65776f726b26636f6c6f723d303030303030266c6f676f3d526f626f742b4672616d65776f726b266c6f676f436f6c6f723d464646464646266c6162656c3d)

## About
The goal of this library is to enable computation of serial kinematics for robotic manipulators in C++. This library is developed as part of the fina project in the course MAE 547: Modelling and Control of Robots at Arizona State University.

## Capabilities

Code was tested on Ubuntu 20.04 with g++ version 9.3.0.

### Completed
- Grubler's Formula for DOF calculation of Serial Link Manipulators
- Model Prismatic and Revolute Joints
- Print DH Tables for Links based on DH Parameters
- Combine Links to Generate a Robot
- Calculate the forward kinematics, degrees of freedom, and print DH Table of a manipulator

### To-Do
- Solve for the manipulator jacobian


## To Use

The source files will need to be included wherever the library is intended for use. To compile and play with the example script written in [main.cpp](https://github.com/Mannat-Rana/Robotics_Toolbox/blob/main/src/main.cpp), follow the below instructions.

```bash
cd ~/git
git clone https://github.com/Mannat-Rana/Robotics_Toolbox.git
cd Robotics_Toolbox
mkdir build
cd build
cmake ..
make
./main
```
