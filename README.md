# Gazebo + PX4 Sim-to-Real Dynamics Validation

无人机动力学仿真验证工具链。使用 Gazebo Harmonic + PX4 v1.16.1 SITL，通过**设定点回放（Setpoint Replay）**方法实现动力学参数的迭代校准。

核心思路：从"真值"飞机的 ULG 日志中提取 trajectory_setpoint，回放给"待校准"的仿真模型，比较两者的动力学响应差异。这套流程与实机验证完全一致——外场飞完拿到 ULG，提取设定点，回放到仿真，对比响应，修正参数。

## 快速开始

```bash
# 1. 安装依赖
pip install pyulog mavsdk matplotlib numpy pandas

# 2. 采集 x500 真值数据
cd PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
# 另一个终端：
python3 scripts/fly_mission.py

# 3. 自动化 4 轮设定点回放实验
bash scripts/run_setpoint_replay_all_rounds.sh

# 4. 或手动单轮：
python3 scripts/update_interceptor_params.py --mass 2.3 --motorConstant 7.01e-06
cd PX4-Autopilot && HEADLESS=1 make px4_sitl gz_interceptor
# 另一个终端：
python3 scripts/setpoint_replay.py \
    --ulg data/flight_logs/x500_truth.ulg \
    --output-dir results/sp_round1

# 5. 提取 ULG 并对比
python3 scripts/extract_ulg.py <interceptor_ulg> --output-dir results/sp_round1/interceptor_csv
python3 scripts/compare.py results/x500_truth_csv results/sp_round1/interceptor_csv \
    --output-dir results/sp_round1

# 6. 查看收敛趋势
python3 scripts/sensitivity.py results/sp_experiments.json
```

## 项目结构

```
gazebo-px4-sim/
├── px4_custom/                         # PX4 中新增的自定义文件
│   ├── models/interceptor/             # interceptor SDF 模型（基于 x500 修改）
│   └── airframes/4050_gz_interceptor
├── PX4-Autopilot/                      # PX4 v1.16.1 源码（.gitignore，需自行克隆）
├── scripts/
│   ├── fly_mission.py                  # x500 真值数据采集（标准机动序列）
│   ├── setpoint_replay.py             # ★ 设定点回放（核心实验脚本）
│   ├── extract_ulg.py                 # ULG → CSV
│   ├── compare.py                     # 时间对齐 + RMSE/R² + 对比图
│   ├── sensitivity.py                 # 多轮收敛分析 + 灵敏度排序
│   ├── sp_convergence_analysis.py     # 4 维收敛图生成
│   ├── update_interceptor_params.py   # 命令行修改 SDF 参数
│   ├── run_setpoint_replay_all_rounds.sh  # 4 轮自动化脚本
│   ├── openloop_replay.py            # 开环执行器回放（实验证伪用）
│   ├── create_hp_model.py            # 创建高性能 x500_hp 模型
│   ├── create_gobi_model.py          # ★ 创建 Gobi 截击机模型（T/W≈10, 97 m/s）
│   ├── fly_multispeed.py             # 多速度段飞行测试（0-50 m/s）
│   ├── fly_gobi_multispeed.py        # ★ Gobi 多速度段飞行（0-97 m/s）
│   ├── gobi_analysis.py              # ★ Gobi 高速分析（k_f + 阻力辨识）
│   ├── fly_sysid_maneuver.py         # 系统辨识激励机动
│   ├── imu_sysid.py                  # IMU-based k_f 辨识
│   ├── analyze_sysid_results.py      # 高速飞行分析
│   ├── comprehensive_analysis.py     # 全速域综合分析
│   ├── compare_ulg.py               # ULG 直接对比
│   ├── compare_replay_aligned.py     # 对齐后 replay 对比
│   ├── run_full_experiment.py        # 高速实验自动化（Python）
│   └── run_highspeed_experiment.sh   # 高速实验自动化（Shell）
├── worlds/
│   └── openloop.sdf                   # 独立 Gazebo world（开环实验用）
├── data/flight_logs/                   # ULG 日志文件（.gitignore）
├── results/
│   ├── sp_round1/ ~ sp_round4/        # ★ 4 轮设定点回放结果
│   ├── highspeed/                     # ★ 高速实验结果（0-50 m/s）
│   │   ├── analysis_x500/            #   x500 基准分析
│   │   ├── analysis_hp_*/            #   HP 模型各速度段分析
│   │   ├── comprehensive/            #   全速域综合分析
│   │   ├── replay_aligned/           #   对齐后 replay 对比
│   │   └── setpoint_replay_*/        #   高速 replay 结果
│   ├── gobi/                          # ★ Gobi 截击机实验结果（0-97 m/s）
│   ├── sysid_analysis/               # 系统辨识结果
│   ├── sp_experiments.json
│   └── x500_truth_csv/
└── README.md
```

## 实验结果（设定点回放）

4 轮迭代参数修正，以速度和姿态 RMSE 为核心指标：

| 轮次 | 修正参数 | Vz RMSE (m/s) | Pitch RMSE (°) | Roll RMSE (°) | Yaw Rate RMSE (rad/s) |
|------|----------|---------------|----------------|---------------|----------------------|
| R1 | 初始猜测 | 0.686 | 4.271 | 5.102 | 0.611 |
| R2 | 质量 | 0.585 (-15%) | 3.794 (-11%) | 5.127 | 0.786 |
| R3 | +推力常数 | **0.357 (-39%)** | **2.332 (-39%)** | **4.221 (-18%)** | **0.357 (-55%)** |
| R4 | +惯性矩+时间常数 | 0.358 | 2.331 | 4.221 | 0.357 |

关键发现：
- **推力系数（motorConstant）是最敏感的参数**，修正后贡献了 ~39% 的 RMSE 下降
- **惯性矩和电机时间常数在设定点回放层面不可观测**——被 PX4 控制器的鲁棒性遮蔽
- **位置 RMSE 因累积漂移不适合作为评价指标**，速度和姿态才是动力学匹配度的真实反映
- **开环执行器回放在四旋翼上不可行**——本征不稳定系统脱离控制器后立即坠毁

## 高速实验结果

### x500_hp（0-50 m/s，T/W≈8）

| 速度段 | k_f 辨识误差 | 诊断含义 |
|--------|------------|---------|
| 10-25 m/s | **3-4%** | 模型结构充分准确，最佳辨识区间 |
| 0-5 m/s | 15-21% | 低信噪比 + 低速阻力扰动 |
| 40-50 m/s | 9-15% | 模型结构开始不足（非线性阻力未建模） |

### Gobi 截击机（0-97 m/s，T/W≈10）

| 速度 | Gazebo 线性阻力 | 真实 v² 阻力 | 倍率 |
|------|---------------|-------------|------|
| 25 m/s | 7.5 N | 7.7 N | 1.0× |
| 50 m/s | 15.0 N | 30.6 N | **2.0×** |
| 97 m/s | 29.1 N | 115.3 N | **4.0×** |

**关键发现**：Gazebo 的 velocity_decay（线性阻力）在 50 m/s 以上完全不可靠，97 m/s 时阻力模型偏差达 4 倍。50+ m/s 截击机仿真必须使用 LiftDrag 插件或自定义 v² 阻力模型。

## 环境要求

- Ubuntu 22.04 (WSL2)
- Gazebo Harmonic (gz-sim 8)
- PX4-Autopilot v1.16.1
- Python 3.10+
- pyulog, mavsdk, matplotlib, numpy, pandas

## 配套文章

[无人机动力学仿真验证实战：从 Gazebo 参数偏差到 4 轮迭代收敛](https://goodisok.github.io/)

## License

MIT
