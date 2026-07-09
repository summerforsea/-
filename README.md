# 水果全自动分拣礼盒一体化机器人系统
针对水果分拣依赖人工、标准不一痛点，本团队基于地瓜RDK X5平台，融合T265视觉与OpenCV，实现自动上料、规格识别、空间定位及机械臂逆解控制，并检测礼盒槽位引导精准入盒。系统集成ROS2与STM32，完成全流程自动化，替代人工，具智慧农业落地潜力。


支持平台型号：RDK X5

支持ROS2(Humbel)版本

前置依赖：需要先安装RealRense SDK，RealRense—ROS，ikpy

RealRense SDK链接：https://github.com/realsenseai/librealsense?tab=readme-ov-file

RealRense—ROS链接：https://github.com/realsenseai/realsense-ros?tab=readme-ov-file

ikpy链接:https://github.com/Phylliade/ikpy

【传送带】

传送带文件包含传送“带传送带拓展版”、“降压模块”以及电机驱动程序，由RDK X5上位机主控，通过通信控制STM32C8T6单片机，驱动传送带电机运转进行上料。


【inter_t265】

inter_t265文件夹包含“test_images”
