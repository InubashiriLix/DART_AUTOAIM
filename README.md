## DART_AUTOAIM (WIP / Pre-alpha)

> RoboMaster 风格自瞄原型（**半成品**）：目前只做了“**亮点**”的快速瞄准原型，**没有**落点预测、**没有**火控、**没有**装甲板识别，整体**不可用于比赛/实装**。  
> **亮点**：主要逻辑用 **C++ 多线程** 实现，尽量**绕开 ROS 消息机制的开销**，以降低端到端延迟。

<p align="left">
  <img src="https://img.shields.io/badge/status-pre--alpha-orange" />
  <img src="https://img.shields.io/badge/lang-C%2B%2B20-blue" />
  <img src="https://img.shields.io/badge/platform-Ubuntu%2022.04%20%7C%20Orange%20Pi%205-lightgrey" />
</p>

---

## TL;DR

- ✅ 已有：相机取流 / **亮点（光源）**快速检测 → 粗略角度 (yaw/pitch) 估计  
- ✅ 特点：**C++ 多线程流水线**，尽量减少 ROS 消息带来的拷贝与调度开销  
- ⚠️ 未验证：曾有人猜测“也许能拿来打基地飞镖”，**但尚未测试**  
- ❌ **大量功能未完成**（见下）

---

## ❌ 现在还没有的功能（重要）

- 落点/弹道预测（空气阻力、飞行时间、仰角/侧偏解算）  
- 火控（发射节律、联动时序、保险/安全逻辑）  
- 装甲板识别（灯条/装甲匹配、ID/数字分类）  
- 车辆类别识别（英雄/步兵/工程/哨兵等）  
- 前哨站与小车的专项逻辑  
- 目标与运动建模（状态机、滤波器体系尚未定稿）  
- **板子切换**（不同上/下位机、串口协议适配）  
- 多目标/多车识别与跟踪（数据关联、丢失重捕）  
- 数据集与标注、回放与评测脚手架  
- 完整的 Launch/参数模板与可复现脚本

> 当前版本**不可用于实战**，仅供原型验证与性能探索。

---

## ✨ 项目亮点 / 设计取向

- **低延迟优先**：核心链路用 **C++ 多线程** 直连，减少跨进程消息与序列化开销  
- **模块化（规划中）**：检测 / 相机几何 / 预测器 / 控制器 分层，后续便于插件化 A/B  
- **可演进**：先把链路打通，再逐步把识别、预测、火控补齐并可切换

---

## 代码结构（会变动，先给个大概）

```

ros\_ws/                 # ROS2 工作区（目前主要用于编译与少量胶水）
src/
camera/             # 相机封装（WIP）
detector/           # 亮点检测原型
geometry/           # 像素 -> yaw/pitch（WIP）
predictor/          # 预测/滤波（占位，未实现落点）
controller/         # 云台/下位机接口（占位）
yolo8\_prototype/        # YOLO 方向的实验草稿（尚未接入实链路）

````

> 目录在快速迭代中；**API/路径随时会改**。

---

## 构建（开发态）

> 以 Ubuntu 22.04 为例。编译只是为了能把 demo 跑起来，**不代表能实战**。

```bash
# 克隆
git clone https://github.com/InubashiriLix/DART_AUTOAIM.git
cd DART_AUTOAIM/ros_ws

# 依赖（按需补充）
sudo apt update
sudo apt install -y build-essential cmake \
  libopencv-dev libeigen3-dev \
  ros-humble-desktop ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-rclcpp ros-humble-sensor-msgs
# 可选：spdlog / toml11 等

# 编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
````

> 💡 如果你在公司/校园网开了代理导致 `git@github.com:22` 连接失败，可把 SSH 改到 `ssh.github.com:443`。

---

## 目前能“跑”的内容

- 单相机 + 亮点检测 demo（WIP）
- 简单的像素坐标 → yaw/pitch 换算（WIP）

> 暂无稳定的 `launch`，请直接看各模块源文件中的 `main()` 或节点入口；随着功能补齐会提供统一入口。

---

## 路线图 / TODO

- [ ] **装甲板识别**（灯条匹配、ID/数字分类、鲁棒性与误检抑制）
- [ ] **落点预测**（含空气阻力、飞行时间、仰角/侧偏解算，延迟补偿）
- [ ] **火控与安全**（发射时序、保险、软硬件互锁）
- [ ] **多目标/多车**（检测→跟踪→关联→优先级策略）
- [ ] **前哨站/小车**专项逻辑
- [ ] **板子切换**与接口抽象（串口/总线/协议适配）
- [ ] **回放评测**（rosbag/视频重放，指标：命中偏差/抖动/耗时）
- [ ] **参数化与 Launch**（可复现实验脚本、标定流程）
- [ ] **性能优化**（锁争用、零拷贝、内存池、SIMD/NEON、GPU/ARM 协同）

---
---

## 声明

- 本项目处于 **半成品 / 实验原型** 阶段；**不要**在任何真实人员/赛场/设备上直接使用。

---
