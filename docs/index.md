# DFL-TORO

**DFL-TORO** is a one-shot demonstration framework for learning time-optimal robotic manufacturing tasks. It optimizes Learning from Demonstration (LfD) by minimizing the need for multiple demonstrations, resulting in efficient, noise-free, and jerk-regulated trajectories. Designed for intuitive kinesthetic demonstrations, DFL-TORO enhances robotic programming efficiency and precision in tasks such as pick-and-place. Evaluations with robots like the Franka Emika Research 3 (FR3) and ABB YuMi demonstrate its effectiveness in transforming demonstrations into optimized, high-performance manufacturing solutions. (Video: https://youtu.be/YJc6DwTqz8o?si=LqTFP7WXPNOFhYCM)

![logo](./images/main.jpg){align=center}

[![arXiv](https://img.shields.io/badge/arXiv-2309.10461-b31b1b.svg)](https://arxiv.org/abs/2309.09802)

## Modules and Packages <a id="modules"></a>

### Principal Modules <a id="principal-modules"></a>

The principal modules comprising DFL-TORO are briefly described in the following table:

| Module                                                    | Description                                                                                                                                                                                                                                                                                                                                                                                   |
| --------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [lfd_interface](https://github.com/snt-arg/lfd_interface) | The LFD Interface module is the core element of DFL-TORO. This module is responsible for providing an interface to various modules of the software and allowing interaction of the user with different functionalities of the software. The main design objective in developing this module was to provide a standard interface to enhance the extensibility and modularity of the framework. |
| [lfd_smoothing](https://github.com/snt-arg/lfd_smoothing) | The main role of the this module is to take raw recorded demonstrations and convert them into optimal demonstration trajectories.                                                                                                                                                                                                                                                             |
| [lfd_dmp](https://github.com/snt-arg/lfd_dmp)             | This module is the implementation of Dynamic Movement Primitives (DMPs), which is responsible for training demonstrations and planning trajectories based on the required start and goal configuration.                                                                                                                                                                                       |

Besides the principal packages, several other packages are included to enable implementation on FR3 and ABB Dual-Arm YuMi.

### FR3 Modules <a id="fr3-modules"></a>

| Module                                                            | Description                                                           |
| ----------------------------------------------------------------- | --------------------------------------------------------------------- |
| [franka_ws](https://github.com/snt-arg/franka_ws)                 | Launch files and required config to run different controllers on FR3. |
| [fr3_moveit_config](https://github.com/snt-arg/fr3_moveit_config) | Moveit configuration package for FR3.                                 |
| [franka_drake](https://github.com/snt-arg/franka_drake)           | Description package to enable using FR3 model in PyDrake.             |

### ABB YuMi Modules <a id="abb-modules"></a>

| Module                                                              | Description                                                                                                                          |
| ------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| [yumi_bringup](https://github.com/snt-arg/yumi_bringup)             | Source files and scripts required to control ABB Yumi via the [ABB Robot Driver](https://github.com/ros-industrial/abb_robot_driver) |
| [yumi_moveit_config](https://github.com/snt-arg/yumi_moveit_config) | Moveit configuration package for ABB Yumi.                                                                                           |
| [yumi_drake](https://github.com/snt-arg/yumi_drake)                 | Description package to enable using Yumi model in [PyDrake](https://drake.mit.edu/).                                                 |
