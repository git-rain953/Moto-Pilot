# 宇树 G1 机器人驾驶摩托车项目集成框架

## 项目概述
本项目旨在集成宇树 G1 人形机器人驾驶摩托车的全流程功能，包括寻找车辆、识别部件、上车交互、稳定骑行及操控驾驶。

## 模块分工与目录结构对应

本框架采用模块化设计，各成员负责的代码模块如下：

| 负责人 | 负责内容 | 对应文件夹/模块 | 核心功能描述 |
| :--- | :--- | :--- | :--- |
| **李浩浩** | 寻找摩托车、导航避障 | `src/navigation/navigator.py` <br> `src/navigation/G1Nav2D/` | 全局路径规划、局部避障、视觉定位摩托车位置 |
| **陈志韬** | 骑上摩托车动作（交互） | `src/interaction/boarder.py` | 上车步态规划、全身协调控制、接触力控制 |
| **刘笑宇** | 识别摩托车按钮、把手 | `src/perception/detector.py` | 细粒度物体识别（油门、刹车、启动键）、6D位姿估计 |
| **玄诗涵** | 骑行稳定性控制 | `src/control/rider.py` <br> (StabilityController) | 动态平衡控制、倒立摆模型/MPC控制、路面适应 |
| **苑天博** | 拧把手、踩油门动作 | `src/control/rider.py` <br> (ManipulationController) | 手臂/腿部逆运动学(IK)、力位混合控制、操作精度控制 |

## 系统架构 (System Architecture)

系统采用有限状态机 (FSM) 进行任务调度：

1. **Idle**: 待机状态
2. **Search & Navigate**: 寻找摩托车并移动到上车点 (李浩浩)
3. **Perception Scan**: 确认把手和按钮位置 (刘笑宇)
4. **Boarding**: 执行上车动作 (陈志韬)
5. **Riding**: 
    - 保持平衡 (玄诗涵)
    - 操控驾驶 (苑天博)

## 目录结构说明

```
G1_Moto_Project/
├── config/                 # 配置文件 (参数、阈值、PID增益)
├── docs/                   # 项目文档 (存放各成员的计划书)
├── scripts/                # 启动脚本
├── src/
│   ├── core/               # 核心架构 (FSM, 消息总线)
│   ├── perception/         
│   │   └── detector.py     # 部件识别与检测 (刘笑宇)
│   ├── navigation/         
│   │   ├── G1Nav2D/        # 导航相关 ROS 包 (李浩浩)
│   │   └── navigator.py    # 导航逻辑封装
│   ├── interaction/        
│   │   └── boarder.py      # 上车交互逻辑 (陈志韬)
│   ├── control/            
│   │   └── rider.py        # 骑行控制与操作 (玄诗涵/苑天博)
│   ├── hardware/           # 硬件接口 (G1 SDK 封装)
│   └── main.py             # 程序入口
└── README.md
```

## 快速开始

1. 安装依赖: `pip install -r requirements.txt`
2. 运行主程序: `python src/main.py`
