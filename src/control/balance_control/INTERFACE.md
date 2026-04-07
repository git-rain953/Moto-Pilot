# 平衡控制模块接口文档

> 模块：G1 人形机器人电动车耦合平衡控制  
> 对接模块：灵巧手模块（负责拧动电动车把手控制车速）  
> 通信协议：Unitree DDS（底层）/ ROS2 话题（模块间）

---

## 一、模块定位

```
┌─────────────────────────────────────────────────────┐
│               灵巧手模块（对接方）                    │
│  功能：控制手指拧动电动车右侧转把，调节车速             │
│                                                     │
│  /dexterous_hand/throttle_cmd  →  当前油门指令       │
│  /dexterous_hand/current_speed ←  当前车速反馈       │
│  /balance/status               ←  平衡状态           │
│  /balance/safe_to_accelerate   ←  是否允许加速       │
└──────────────────┬──────────────────────────────────┘
                   │ ROS2 话题
┌──────────────────▼──────────────────────────────────┐
│           平衡控制模块（本模块）                       │
│  功能：控制腰部 + 手臂关节维持人车整体平衡              │
│  输入：IMU + 关节编码器 + 当前车速                     │
│  输出：10 个关节 PD 目标位置（DDS LowCmd）             │
└─────────────────────────────────────────────────────┘
                   │ Unitree DDS
┌──────────────────▼──────────────────────────────────┐
│           G1 机载 MCU                                │
│  rt/lowstate（500Hz）← 关节状态 + IMU                │
│  rt/lowcmd  （50Hz） → 关节控制指令                   │
└─────────────────────────────────────────────────────┘
```

---

## 二、与灵巧手模块的 ROS2 对接接口

### 2.1 本模块发布（灵巧手需订阅）

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/balance/status` | `std_msgs/String` | 10 Hz | 平衡控制状态，枚举值见下表 |
| `/balance/safe_to_accelerate` | `std_msgs/Bool` | 10 Hz | `true` = 当前平衡良好，允许加速；`false` = 请减速/停止油门 |
| `/balance/roll_pitch` | `geometry_msgs/Vector3` | 50 Hz | 实时 roll/pitch（rad），灵巧手可用于判断倾斜程度 |

**`/balance/status` 枚举：**

| 值 | 含义 | 灵巧手应对措施 |
|----|------|--------------|
| `WARMING_UP` | 热身阶段（2s），关节从阻尼过渡到控制 | 保持油门为 0 |
| `RUNNING` | 平衡控制运行中 | 正常控制油门 |
| `FALLEN` | 检测到倾倒（\|roll\| > 45° 或 \|pitch\| > 60°） | 立即松开油门 |
| `STOPPED` | 脚本已停止 | 立即松开油门 |

### 2.2 本模块订阅（需灵巧手发布）

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/dexterous_hand/current_speed` | `std_msgs/Float32` | 10 Hz | 当前电动车实际速度（m/s），作为平衡策略观测输入 |

**字段说明：**

```python
msg.data   # 当前车速，单位 m/s，范围 [0.0, ~5.0]
           # 平衡控制策略内部归一化：obs[26] = speed / 3.0
```

> 若灵巧手模块暂未实现速度反馈，可发布固定值 0.0，平衡策略仍可运行（静止平衡模式）。

---

## 三、协作流程

```
启动顺序：
  1. 机器人进入阻尼模式（遥控器 L2+A）
  2. 启动平衡控制模块（deploy_real.py）
  3. 等待 /balance/status == "RUNNING"（约 2 秒热身）
  4. 灵巧手模块开始控制油门

运行中：
  - 灵巧手实时订阅 /balance/safe_to_accelerate
  - 若收到 false，立即将转把归零（松油门）
  - 平衡控制模块自动根据车速调整平衡策略

停止顺序：
  1. 灵巧手先将油门归零（车速降至 0）
  2. 再停止平衡控制模块（Ctrl+C）
  3. 机器人自动切换为纯阻尼状态
```

---

## 四、DDS 底层接口（仅供参考，平衡控制内部使用）

### 订阅

| DDS Topic | 消息类型 | 频率 | 说明 |
|-----------|----------|------|------|
| `rt/lowstate` | `unitree_hg/LowState_` | 500 Hz | IMU + 全部 29 关节状态 |

```python
LowState_.imu_state.gyroscope[3]    # 角速度 (rad/s)
LowState_.imu_state.quaternion[4]   # 四元数 [w,x,y,z]
LowState_.imu_state.rpy[3]          # roll/pitch/yaw (rad)
LowState_.motor_state[idx].q        # 关节角度 (rad)
LowState_.motor_state[idx].dq       # 关节速度 (rad/s)
```

### 发布

| DDS Topic | 消息类型 | 频率 | 说明 |
|-----------|----------|------|------|
| `rt/lowcmd` | `unitree_hg/LowCmd_` | 50 Hz | PD 位置控制指令 |

```python
LowCmd_.mode_machine = 3
LowCmd_.motor_cmd[idx].q    # 目标角度 (rad)
LowCmd_.motor_cmd[idx].kp   # 位置增益
LowCmd_.motor_cmd[idx].kd   # 速度增益
LowCmd_.crc = crc.Crc(cmd)  # ⚠️ 必须设置
```

---

## 五、控制关节列表（10 个）

| 索引 | 关节名 | Motor ID | 作用 | PD 增益 (kp/kd) |
|------|--------|----------|------|-----------------|
| 0 | waist_yaw | 12 | 腰部左右扭转 | 50 / 5 |
| 1 | waist_roll | 13 | **主平衡关节**，身体侧倾 | 50 / 5 |
| 2 | waist_pitch | 14 | 身体前后倾 | 50 / 5 |
| 3 | L_shoulder_pitch | 15 | 左臂前后 | 15 / 2 |
| 4 | L_shoulder_roll | 16 | 左臂展开 | 15 / 2 |
| 5 | L_shoulder_yaw | 17 | 左臂旋转 | 15 / 2 |
| 6 | L_elbow | 18 | 左肘弯曲 | 15 / 2 |
| 7 | R_shoulder_pitch | 22 | 右臂前后 | 15 / 2 |
| 8 | R_shoulder_roll | 23 | 右臂展开 | 15 / 2 |
| 9 | R_elbow | 25 | 右肘弯曲 | 15 / 2 |

> 腿部 12 个关节：kp=0，kd=2（纯阻尼，维持坐姿）  
> 手腕 6 个关节：kp=30，kd=3（锁定，握住车把）

---

## 六、骑行参考姿态（标定值，2026-03-27）

机器人坐在车上、手握车把时的关节偏置：

| 关节 | Motor ID | 偏置 (rad) | 偏置 (°) |
|------|----------|-----------|---------|
| waist_yaw | 12 | -0.0835 | -4.8° |
| waist_roll | 13 | -0.0547 | -3.1° |
| waist_pitch | 14 | -0.0719 | -4.1° |
| L_shoulder_pitch | 15 | +0.1920 | +11.0° |
| L_shoulder_roll | 16 | +0.1360 | +7.8° |
| L_shoulder_yaw | 17 | -0.5021 | -28.8° |
| L_elbow | 18 | +1.2655 | +72.5° |
| R_shoulder_pitch | 22 | +0.1950 | +11.2° |
| R_shoulder_roll | 23 | -0.0726 | -4.2° |
| R_elbow | 25 | +1.2858 | +73.7° |

---

