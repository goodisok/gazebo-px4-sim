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
│   ├── extract_ulg.py                 # ULG → CSV（位置/速度/姿态/角速度/电机）
│   ├── compare.py                     # 时间对齐 + RMSE/R² + 对比图
│   ├── sensitivity.py                 # 多轮收敛分析 + 灵敏度排序
│   ├── sp_convergence_analysis.py     # 4 维收敛图生成
│   ├── update_interceptor_params.py   # 命令行修改 SDF 参数
│   ├── run_setpoint_replay_all_rounds.sh  # 4 轮自动化脚本
│   └── openloop_replay.py            # 开环执行器回放（实验证伪用）
├── worlds/
│   └── openloop.sdf                   # 独立 Gazebo world（无 PX4，开环实验用）
├── data/flight_logs/                   # ULG 日志文件（.gitignore）
├── results/
│   ├── sp_round1/ ~ sp_round4/        # ★ 4 轮设定点回放结果（CSV + 对比图 + metrics）
│   ├── sp_experiments.json            # 设定点回放实验配置
│   ├── sp_convergence_full.png        # 4 维收敛图
│   ├── x500_truth_csv/               # 真值数据 CSV
│   ├── round1/ ~ round4_fix_inertia/  # 旧版同脚本实验结果（留作参考）
│   ├── openloop_round4/              # 开环回放结果（证伪实验）
│   ├── convergence.png
│   └── sensitivity_table.txt
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
