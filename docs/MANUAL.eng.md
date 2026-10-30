<div align="center" style="text-align: center;">

# Frob Robotic Platform

![Frob Logo](https://github.com/user-attachments/assets/4e6b15c5-6662-47f5-8419-bada88828910)

[Russian-language documentation](MANUAL.md) · [Team repository](../README.md) · [Competition solution guide (Russian)](SOLUTION.md)

**Frob** is an open—source low cost mobile robot created to promote robotics and [ROS](https://www.ros.org/) training. It is designed for enthusiasts, students, and teachers exploring the world of robotics. Whether you are a beginner or an experienced developer, Frob provides a versatile platform for addressing a wide range of learning challenges.

[Frob Wiki](https://github.com/dark516/Frob_robot/wiki) · [Frob community](https://t.me/FrobCommunity)

---

</div>

## Table of Contents
- [Installation](#installation)
- [Fabrication and Assembly](#fabrication-and-assembly)
- [Basic Usage](#basic-usage)
- [Contribution Guidelines](#contribution-guidelines)
- [Community](#community)
- [License](#license)

---

## Installation
To get started with this project, follow the steps below.
### 1. Clone the Repository
Start by cloning this repository to your local machine:
```bash
git clone https://github.com/romazanovart/RTK_Gorod.git
cd RTK_Gorod
```

### 2. Install ROS2
This project requires ROS2 (we recommend using ROS-Jazzy). There are two options to install ROS2-jazzy:
#### Option A: Install ROS2 using the provided scripts
In the [installation](../installation) directory you will find scripts for installing ROS2 Jazzy along with the necessary dependencies for different computers. To use the scripts, run one of the following commands, depending on the device to install:
##### Install ROS2 on the Workstation (Laptop or PC)
```bash
bash ./installation/install_jazzy_full.sh
```
##### Install ROS2 on the Raspberry pi (Robot)
```bash
bash ./installation/install_jazzy_base.sh
```
#### Option B: Manual Installation of ROS2
If you'd prefer to install ROS2 manually, please refer to the official ROS2 installation guide [here](https://docs.ros.org/en/jazzy/Installation.html). This guide provides step-by-step instructions to set up ROS2 Jazzy on various platforms.

## Fabrication and Assembly

The Frob platform guide includes a [parts list](assembly_guide/FROB_parts_and_prices.xlsx) and an [illustrated assembly guide](assembly_guide/assembly_guide.png). See the [Russian platform manual](MANUAL.md#изготовление-и-сборка) for chassis, top plate and power-board references.

## Basic Usage

The competition solution combines a camera-based sign detector, road-marking analysis and motor control through Arduino. The [solution guide (Russian)](SOLUTION.md) covers dependencies, building the ROS 2 workspace, launch commands, mission services and debug video streams.

The supplied weights are included in the repository. The detector uses a configurable Python interpreter, so the project can be cloned outside the original Raspberry Pi home directory.

## Contribution Guidelines

Describe the affected hardware, software versions, reproduction steps and expected behaviour when reporting a problem. For code changes, explain the resulting behaviour and the checks performed. Keep robot calibration and track-specific settings explicit.

## Community

Frob is an actively growing project, and we warmly invite you to join our community. Whether you encounter issues or are looking for fresh ideas, our community is here to support you.

- **[Join our Telegram Community Chat](https://t.me/FrobCommunity)**: Connect with other Frob users, ask questions, share your experiences, and stay updated on the latest developments.

Your participation can help shape the future of Frob. We look forward to your ideas and contributions!

## License

Frob is licensed under the [Apache License 2.0](../LICENSE).
For a detailed understanding of your rights and responsibilities, please refer to the [full license text](../LICENSE).
