# G1 人形机器人骑行平衡控制 — 部署包

## 模块说明

本模块实现 **Unitree G1 人形机器人与电动自行车耦合系统的平衡控制**，基于 PPO 强化学习策略，运行于 G1 机载 PC（Ubuntu 20.04，Python 3.8）。

---

## 文件结构

```
g1_bike_deploy/
├── deploy_real.py          # 主部署脚本
├── models/
│   ├── ppo_riding_v4_actor.pt       # TorchScript 推理模型（无需 SB3）
│   └── ppo_riding_v4_norm.json      # 观测归一化参数（obs_mean/obs_var）
└── README.md
```

---

## 环境依赖

在 G1 机载 PC 上安装：

```bash
pip3 install torch --index-url https://download.pytorch.org/whl/cpu
pip3 install numpy

# Unitree SDK2（已预装）
# unitree_sdk2py + cyclonedds==0.10.2
```

---

## 快速运行

```bash
# 1. 干跑测试（无需机器人）
python3 deploy_real.py --dry-run \
  --model models/ppo_riding_v4_actor.pt \
  --norm  models/ppo_riding_v4_norm.json

# 2. 实机运行（机器人需先进入阻尼/调试模式：遥控器 L2+A）
python3 deploy_real.py \
  --model   models/ppo_riding_v4_actor.pt \
  --norm    models/ppo_riding_v4_norm.json \
  --network eth0 \
  --speed   0.0

# 3. 急停：Ctrl+C 或 kill -SIGINT <pid>
```

---

## 安全须知

> **每次运行前必须确认：**
> 1. 机器人已绑好安全吊绳
> 2. 机器人已就座于电动车上，手握车把
> 3. 机器人处于 **阻尼/调试模式**（L2+A），关节已变软
> 4. 旁边有人随时准备按遥控急停

---

## 接口说明

详见 [INTERFACE.md](INTERFACE.md)
