# Fully Automated Fruit Sorting & Gift Box Integration Robotic System

Addressing industry pain points such as high dependency on manual labor and inconsistent sorting standards, our team has developed an integrated, fully automated sorting robotic system. Based on the **Horizon Robotics RDK X5 embedded development platform**, the system fuses **Intel RealSense T265 visual tracking technology** with **OpenCV image processing** to achieve automated feeding, specification identification, spatial localization, robotic arm inverse kinematics control, and precise box slot guidance.

The system seamlessly integrates **ROS2** with an **STM32** MCU for down-level execution, realizing full-process automation. Designed to replace traditional manual workflows with intelligent automation, it holds immense potential for real-world smart agriculture applications.

---

## 🛠️ Core Technology Stack & Platform Support

*   **Host Control Platform**: Horizon Robotics RDK X5
*   **Microcontroller (MCU)**: STM32C8T6 / STM32 series core board
*   **Distributed System Framework**: ROS2 (Humble Hawksbill)
*   **Core Algorithm Libraries**: OpenCV (Python), YOLOv11 Object Detection, Robotic Arm Inverse Kinematics (`ikpy`)

---

## 📦 Environment Deployment & Prerequisites

Before compiling or running this system, please ensure the host side is configured with the following SDKs and underlying dependencies:

1.  **RealSense SDK (librealsense)**
    *   Used to drive the underlying data streams of the T265 camera: [Official GitHub Link](https://github.com/realsenseai/librealsense?tab=readme-ov-file)
2.  **RealSense — ROS2 Driver**
    *   Bridges image and tracking data to ROS2 nodes: [Official GitHub Link](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file)
3.  **Python Dependencies**
    *   `opencv-python`: Used for visual undistortion and image preprocessing.
    *   `ikpy`: Used for 6-axis robotic arm inverse kinematics resolution: [Official GitHub Link](https://github.com/Phylliade/ikpy)

---

## 📂 Project Directory Structure & Modules

The project adopts a modular design where each core component carries distinct responsibilities. Below is the main directory layout:

```text
├── inter_t265/           # Visual Processing & AI Inference Module
│   ├── test_images/      # Test dataset image folder
│   ├── t265_yolo11.bin   # YOLOv11 visual model optimized for the on-board BPU
│   └── t265_yolo.py      # Core execution source code (image capture, undistortion, & inference)
│
├── Conveyor_Belt/        # Automated Feeding Hardware & Control Module
│   ├── 12降5V.epro2      # 12V to 5V step-down module LCEDA project file
│   ├── 12降5v.png        # Step-down module schematic / PCB screenshot
│   ├── 传送带拓展版.epro2  # Conveyor belt extension board LCEDA project file
│   ├── 传送带拓展版.png    # Conveyor belt extension board schematic / PCB screenshot
│   └── chuansongdai.zip  # Motor driver and MCU core control source code
│
├── Robotic_Arm/          # Precision Sorting & Gripping Execution Module
│   ├── [Robotic Arm Source] # ROS2-based motion planning and control nodes
│   ├── [IK_Scripts]      # Motion trajectory calculation scripts utilizing ikpy
│   └── [Hardware_Files]  # Robotic arm structural design and driver board engineering files
│
├── .gitignore            # Git ignore configuration file
└── README.md             # Main project documentation (System Master Control Guide)

Here is the professional English translation for the individual module descriptions you provided. You can use these descriptions directly for individual module documentation or presentation slides:

【Conveyor_Belt】
The conveyor belt directory contains the "Conveyor Belt Extension Board" configuration, "Step-down Module" engineering files, and underlying motor driver routines. Controlled by the RDK X5 host as the master processor, the system dispatches instructions via serial/bus communication to the low-level STM32C8T6 MCU, driving the conveyor motor to execute automated feeding.

【inter_t265】
The inter_t265 directory encompasses the test_images validation dataset, the t265_yolo11.bin visual model file, and the t265_yolo.py core execution source code. Frame images are captured via the T265 camera, followed by OpenCV-driven preprocessing to eliminate lens distortion. The rectified, crisp frames are then fed into the board on-the-fly, leveraging the hardware BPU for real-time AI inference.

【Robotic_Arm】
This directory comprises the ROS2-based motion planning and control nodes, alongside trajectory calculation core files. It handles real-time inter-module communication through the RDK X5 host, collaborating seamlessly with other components to execute and complete the overall target tasks.
