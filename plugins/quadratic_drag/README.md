# QuadraticDrag — v² 机身阻力 (Gazebo Sim 8)

对 `base_link` 施加 **F = −½ρ·CdA·|v|·v**（世界系），用于高速四旋翼在 Gazebo 中替代不物理的 `velocity_decay`。

## 构建

```bash
cd plugins/quadratic_drag
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

得到 `libQuadraticDrag.so`。

## 运行 PX4+Gazebo

在启动仿真前设置：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="/path/to/gazebo-px4-sim/plugins/quadratic_drag/build"
```

SDF 中在 `<model>` 内、电机插件之后加入：

```xml
<plugin filename="QuadraticDrag" name="quadratic_drag::QuadraticDrag">
  <link_name>base_link</link_name>
  <CdA>0.020</CdA>
  <air_density>1.225</air_density>
</plugin>
```

- `CdA`：阻力系数与参考面积的乘积 (m²)，需由风洞/实飞/估计标定。
- `air_density`：海平面约 1.225 kg/m³，可按高度修正。

**不要** 与 `velocity_decay` 同时加在同一机体上，以免重复计阻。

## 与 PX4 的关系

本插件在 Gazebo 物理步中对机体施力，PX4 的 `MPC_*` 与电机模型仍照常工作，适合 setpoint replay 与参数辨识。
