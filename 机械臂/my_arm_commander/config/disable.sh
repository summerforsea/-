#!/bin/bash
# ============================================================
# 六轴机械臂 — 电机失能（停止）脚本
# 基于 42Motor CAN 通信协议：命令 0x01，数据 0x00 = 停止
# CAN ID = (NodeID << 7) | 0x01
# Node ID 1~6 分别对应 joint1 ~ joint6
# ============================================================

CAN_IF="can0"

echo "正在停止六个电机..."

# joint1 — Node ID 1 → CAN ID = (1 << 7) | 0x01 = 0x081
cansend ${CAN_IF} 081#0000000000000000
sleep 0.01

# joint2 — Node ID 2 → CAN ID = (2 << 7) | 0x01 = 0x101
cansend ${CAN_IF} 101#0000000000000000
sleep 0.01

# joint3 — Node ID 3 → CAN ID = (3 << 7) | 0x01 = 0x181
cansend ${CAN_IF} 181#0000000000000000
sleep 0.01

# joint4 — Node ID 4 → CAN ID = (4 << 7) | 0x01 = 0x201
cansend ${CAN_IF} 201#0000000000000000
sleep 0.01

# joint5 — Node ID 6 → CAN ID = (6 << 7) | 0x01 = 0x301
cansend ${CAN_IF} 301#0000000000000000
sleep 0.01

# joint6 — Node ID 5 → CAN ID = (5 << 7) | 0x01 = 0x281
cansend ${CAN_IF} 281#0000000000000000

echo "六个电机已全部停止"
