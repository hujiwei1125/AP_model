# AP闭环测试简化六自由度运动学模型

## 1. 模型定位

该模型仅用于 `AP_v8(3)/GuidanceLaw.slx` 的逻辑和功能闭环验证，不用于验证真实飞机气动特性、控制律稳定裕度、执行机构或控制分配。

`GuidanceLaw` 已对输出指令完成幅值和变化率保护，因此模型直接使用 `apCmd_struct`，不再增加速度、加速度、jerk和姿态响应参数。

模型保留以下必要关系：

- 三轴位置由NEU速度积分获得；
- 旋翼速度指令由机体系转换到NEU坐标系；
- 旋翼姿态角直接跟踪AP姿态角指令；
- 固定翼和倾转阶段根据滚转角计算协调转弯角速度；
- 速度和姿态相邻拍差分得到加速度与角速度；
- 绝对海拔和离地高度分别输出；
- 输出总线与 `apTransNavBus` 一致。

## 2. 函数接口

```matlab
[nav_struct, posNEU_m_f, velNEU_mps_f, accNEU_mps2_f, attEuler_deg_f, rateBody_dps_f] = simple6DofPlant(apCmd_struct, flightPhase_enum, reset_b, dt_s_f, initState_f, refLat_deg_d, refLon_deg_d, refAlt_m_f, groundAlt_m_f)
```

| 输入 | 类型和尺寸 |
| --- | --- |
| `apCmd_struct` | `Bus: apCmdBus` |
| `flightPhase_enum` | `Enum: FLIGHT_PHASE`或对应的`uint8` |
| `reset_b` | `boolean`单拍复位信号 |
| `dt_s_f` | `single`模型运行周期 |
| `initState_f` | `single(12x1)`初始状态 |
| `refLat_deg_d` | `double`参考纬度 |
| `refLon_deg_d` | `double`参考经度 |
| `refAlt_m_f` | `single`参考点绝对海拔 |
| `groundAlt_m_f` | `single`地面绝对海拔 |

`initState_f`定义为：

```text
[N; E; U; Vn; Ve; Vu; roll; pitch; yaw; p; q; r]
```

## 3. AP输出的使用方式

| AP输出 | 运动学模型处理方式 |
| --- | --- |
| `vxCmd_mps_f` | 旋翼阶段为机体前向速度；倾转/固定翼阶段为真空速 |
| `vyCmd_mps_f` | 旋翼阶段为机体右向速度；倾转/固定翼阶段不使用 |
| `vuCmd_mps_f` | NEU向上速度 |
| `rollCmd_deg_f` | 直接作为滚转角；倾转/固定翼阶段用于计算转弯率 |
| `pitchCmd_deg_f` | 直接作为俯仰角 |
| `yawCmd_deg_f` | 旋翼阶段直接作为偏航角；倾转/固定翼阶段不直接跟踪 |
| `apRunState_b` | 为假时保持当前状态 |

旋翼阶段速度转换关系为：

```matlab
Vn = cos(yaw) * vx - sin(yaw) * vy;
Ve = sin(yaw) * vx + cos(yaw) * vy;
Vu = vu;
```

倾转和固定翼阶段使用：

```matlab
yawRate = g * tan(roll) / trueAirspeed;
```

当空速不大于 `1 m/s` 时将转弯率置零，只用于避免后倾接近停车时除零。

## 4. 高度定义

```matlab
alt_m_f = refAlt_m_f + U;
hag_m_f = max(alt_m_f - groundAlt_m_f, 0);
```

因此航线可以继续使用绝对海拔高度，垂直起降和模式切换可以使用 `hag_m_f`。

## 5. Simulink集成

1. 将本目录加入MATLAB路径，运行 `initSimple6DofPlant.m`。
2. 在 MATLAB Function 模块中调用 `simple6DofPlant`。
3. 输入 `apCmd_struct` 类型设置为 `Bus: apCmdBus`。
4. 输出 `nav_struct` 类型设置为 `Bus: apTransNavBus`。
5. 将 `GuidanceLaw` 的 `apCmd_struct` 接入模型，并将模型的 `nav_struct` 反馈到 `GuidanceLaw`。
6. 推荐运动学模型与 GuidanceLaw 使用相同的 `0.05 s` 周期，避免离散速度指令差分得到不必要的加速度尖峰。

函数采用“本拍输出当前状态、再更新下一拍状态”的结构，`apCmd_struct` 到 `nav_struct` 没有直接馈通，正常情况下不会形成代数环。

## 6. 文件

- `simple6DofPlant.m`：主运动学模型。
- `initSimple6DofPlant.m`：采样周期、地理参考和初始状态。
- `demo_simple6DofPlant.m`：分阶段开环测试脚本。
