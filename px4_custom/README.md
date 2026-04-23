# PX4 自定义文件

本目录包含在 PX4 v1.16.1 源码中新增/修改的文件。由于 PX4-Autopilot 体积过大（~1GB），
仓库中不包含完整的 PX4 源码，而是将自定义部分提取到这里。

## 文件说明

```
px4_custom/
├── models/
│   └── interceptor/
│       ├── model.sdf        # 自定义 interceptor 四旋翼 SDF 模型（基于 x500 修改）
│       └── model.config      # Gazebo 模型元数据
├── airframes/
│   └── 4050_gz_interceptor   # PX4 airframe 初始化脚本
└── README.md
```

## 安装步骤

1. 克隆 PX4 v1.16.1：

```bash
git clone --branch v1.16.1 --recursive https://github.com/PX4/PX4-Autopilot.git
```

2. 复制 interceptor 模型到 Gazebo 模型目录：

```bash
cp -r px4_custom/models/interceptor \
  PX4-Autopilot/Tools/simulation/gz/models/interceptor
```

3. 复制 airframe 配置：

```bash
cp px4_custom/airframes/4050_gz_interceptor \
  PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

4. 在 `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 中，
   找到 `4021_gz_x500_flow` 那一行，在其后添加：

```cmake
	4050_gz_interceptor
```

5. 编译并运行：

```bash
cd PX4-Autopilot
HEADLESS=1 make px4_sitl gz_interceptor
```
