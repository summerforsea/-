import ikpy.chain
import numpy as np
import matplotlib.pyplot as plt

# 1. 载入你的 URDF (路径换成你真实的路径)
urdf_path = "/home/sunrise/YYC/jixiebi/jixiebi_ws/src/my_arm_hardware/urdf/jixiebi.urdf"
arm_chain = ikpy.chain.Chain.from_urdf_file(urdf_path,
                                            active_links_mask=[False, True, True, True, True, True, True])

# 2. 打印看看 ikpy 到底解析出了几个 link？（极其关键！）
print(f"URDF 解析出的活动关节数: {len(arm_chain.links)}")
for i, link in enumerate(arm_chain.links):
    print(f"索引 {i}: {link.name}")

# 3. 设置目标坐标
target_position = [0.1, 0.1, 0.2]

# 4. 算 IK
ik_solution = arm_chain.inverse_kinematics(target_position)
print(f"完整的 ik_solution 数组: {ik_solution}")

# 5. 画出 3D 骨架图并保存
fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
arm_chain.plot(ik_solution, ax, target=target_position)

# 设置一下视角，方便观察
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 0.6)

# 保存图片到当前目录
plt.savefig("ik_debug_plot.png")
print("骨架图已保存为 ik_debug_plot.png，请查看！")