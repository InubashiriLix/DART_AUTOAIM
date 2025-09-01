# temp_ui — Terminal UI Prototype (WIP)

> 这是 DART_AUTOAIM 项目里的一个**纯终端 TUI 测试版**，用来快速查看各模块的运行状态并验证**低延迟多线程链路**。项目整体定位见仓库根部 README（RoboMaster 风格自瞄原型，当前仍是半成品）。  

## 功能概览

- 四栏窗口：**Detector / Camera / Kalman / Contact**，外加顶部 **Command** 栏。
- **LatestChannel** + `shared_ptr<const map<string,string>` 数据模型，发布者整张表替换，消费者只读，避免撕裂。
- **快/慢路径刷新**：
  - 键集合不变 → 仅更新值（更少闪烁）。
  - 键集合变化 → 重建键与布局。
- 三个工作线程：
  - `inputThreadWorker`：非阻塞按键输入（SPSC 队列）。
  - `windowsThreadWorker`：渲染循环（含最小终端尺寸检查）。
  - `infoUpdateThreadWorker`：从各通道轮询并更新窗口。

## 键位

- 在 **Command** 栏：  
  - `D / C / K / c` 切到对应窗口  
  - `t` 单窗/四窗切换  
  - `Q` 退出（发退出信号）
- 在内容窗口：  
  - `j / k` 上下移动  
  - `q` 返回 Command

## 目录与主要文件（简述）

- `Tui.hpp`：线程与窗口调度、键集合比较（C++20 `std::ranges::equal`）。
- `Window.hpp`：窗口模型；提供 `setDisplayMap(...)`（重建）与 `setDisplayMapValuesOnly(...)`（仅更新值）。
- `Terminal.hpp`：简单终端渲染/无阻塞输入（基于 ANSI 序列，具体实现见源码）。
- `LatestChannel.hpp`：单写多读的“最新值通道”。

> 代码仍在变动，文件名/接口以后可能调整（以源码为准）。

