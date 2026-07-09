#!/bin/bash
# ============================================================
# 六轴机械臂 — 设置零位（归位）脚本
# 基于 42Motor CAN 通信协议：命令 0x15，将当前位置记为零点
# 自动保存到 EEPROM，掉电不丢失
# CAN ID = (NodeID << 7) | 0x15
# Node ID 1~6 分别对应 joint1 ~ joint6
# ============================================================

CAN_IF="can0"

echo "正在将六个电机当前位置设为零位..."

# joint1 — Node ID 1 → CAN ID = (1 << 7) | 0x15 = 0x095
cansend ${CAN_IF} 095#0000000000000000
sleep 0.01

# joint2 — Node ID 2 → CAN ID = (2 << 7) | 0x15 = 0x115
cansend ${CAN_IF} 115#0000000000000000
sleep 0.01

# joint3 — Node ID 3 → CAN ID = (3 << 7) | 0x15 = 0x195
cansend ${CAN_IF} 195#0000000000000000
sleep 0.01

# joint4 — Node ID 4 → CAN ID = (4 << 7) | 0x15 = 0x215
cansend ${CAN_IF} 215#0000000000000000
sleep 0.01

# joint5 — Node ID 6 → CAN ID = (6 << 7) | 0x15 = 0x315
cansend ${CAN_IF} 315#0000000000000000
sleep 0.01

# joint6 — Node ID 5 → CAN ID = (5 << 7) | 0x15 = 0x295
cansend ${CAN_IF} 295#0000000000000000

echo "六个电机零位已设置并保存到 EEPROM"
