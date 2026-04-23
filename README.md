# Gazebo + PX4 Sim-to-Real Dynamics Validation

无人机动力学仿真验证工具链。使用 Gazebo Harmonic + PX4 v1.16.1 SITL，通过自动化飞行测试和数据对比，实现动力学参数的迭代校准。

## 快速开始

```bash
# 1. 安装依赖
pip install pyulog mavsdk matplotlib numpy pandas

# 2. 运行 x500 (真值) 仿真
cd PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500

# 3. 在另一个终端运行飞行任务
cd scripts
python3 fly_mission.py

# 4. 提取 ULG 数据
python3 extract_ulg.py <path_to_ulg> --output-dir data/x500_csv

# 5. 对比
python3 compare.py data/x500_csv data/interceptor_csv --output-dir results/
```

## 项目结构

```
gazebo-px4-sim/
├── PX4-Autopilot/              # PX4 v1.16.1 (git submodule)
│   └── Tools/simulation/gz/models/
│       └── interceptor/        # 自定义 interceptor 模型
├── scripts/
│   ├── fly_mission.py          # 自动化标准机动飞行
│   ├── extract_ulg.py          # ULG 日志解析 → CSV
│   ├── compare.py              # 数据对比 + RMSE + 图表
│   ├── sensitivity.py          # 参数灵敏度分析
│   ├── update_interceptor_params.py  # 修改模型参数
│   └── run_experiment.sh       # 自动化实验脚本
├── data/flight_logs/           # ULG 日志文件
├── results/                    # 对比结果、图表、指标
│   ├── round1/                 # 初始猜测
│   ├── round2_fix_mass/        # 修正质量
│   ├── round3_fix_motor/       # 修正推力常数
│   ├── round4_fix_inertia/     # 修正惯性矩
│   ├── convergence.png         # RMSE 收敛图
│   └── experiments.json        # 实验配置
└── README.md
```

## 实验结果

| 轮次 | 修正参数 | Z RMSE (m) | Roll RMSE (°) | Vz RMSE (m/s) |
|------|----------|------------|----------------|----------------|
| Round 1 | 初始猜测 | 0.706 | 4.585 | 0.350 |
| Round 2 | 质量 | 0.171 | 3.662 | 0.245 |
| Round 3 | +推力常数 | 0.288 | 4.868 | 0.253 |
| Round 4 | +惯性矩+时间常数 | 0.147 | 3.436 | 0.113 |

## 环境要求

- Ubuntu 22.04 (WSL2)
- Gazebo Harmonic
- PX4-Autopilot v1.16.1
- Python 3.10+
- pyulog, mavsdk, matplotlib, numpy, pandas

## License

MIT
