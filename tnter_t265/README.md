# inter_t265 视觉处理与推理模块

本模块是“水果全自动分拣礼盒一体化机器人系统”的**核心视觉感知组件**。它利用 Intel RealSense T265 追踪摄像头获取鱼眼图像，通过 OpenCV 进行去畸变处理，并结合板载 BPU 芯片实现水果的高效实时目标检测（YOLO）与推理。

---

## ⚙️ 硬件与环境支持

*   **支持平台型号**：地瓜 RDK X5（利用其自带的 BPU 算力进行实时推理）
*   **支持 ROS2 版本**：Humble

---

## 🛠️ 前置依赖

在运行本模块前，请确保系统已安装以下 SDK 与依赖库：

*   **RealSense SDK (librealsense)**：[GitHub 链接](https://github.com/realsenseai/librealsense?tab=readme-ov-file)
*   **RealSense—ROS 驱动**：[GitHub 链接](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file)
*   **OpenCV** (opencv-python)

---

## 🚀 工作原理与运行流程

1.  **图像采集**：通过连接的 T265 摄像头截取双目/鱼眼帧图像。
2.  **图像预处理**：由于 T265 原生输出为畸变较大的鱼眼图像，系统通过 `open cv` 对畸变图像进行**去畸变处理**。
3.  **BPU 实时推理**：将处理后的清晰图像实时导入到 RDK X5 板端，利用板载 BPU 硬件加速运行 `t265_yolo11.bin` 模型，实现水果规格识别与空间定位。

---

## 📂 目录结构说明

```text
├── test_images/          # 测试数据集图片文件夹（用于离线测试或验证）
├── t265_yolo11.bin       # 针对 BPU 编译优化的 YOLOv11 视觉模型文件
├── t265_yolo.py          # 核心执行源码（包含图像获取、去畸变及推理调用）
└── README.md             # 本说明文档
