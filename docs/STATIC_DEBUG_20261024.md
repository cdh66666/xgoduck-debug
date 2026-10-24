# 上电静止调试 · 2026-10-24

用户确认鸭子仍在托盘，只做静止调试。当前继续卸力并保留 `motion-inhibit`；没有请求上力、站立、行走或机械校准。本轮未修改训练代码或模型权重。

## 当前代码链路

`Start-Duck.ps1` 安装依赖并导入连接身份 → `pc/server.py` / `factory_workbench.py` 启动调试台 → `official_control_proxy.py` 发现并核对主控、建立 SSH 转发 → `official_status_stream.py` 读取状态与判断时效 → `robot-app/python/main.py` 处理认证及禁动 → `controller.py` 接收/滤波、运行策略和发送命令 → `sketch/sketch.ino`、QMI 驱动、SCS 总线操作实际 MCU 与舵机。

模型的 ONNX 计算、`policy.py`、`keyboard_runtime.py`、`recovery.py` 在主控本地；电脑画面晚到不能单独证明模型推理晚到。电流与动作记录独立保存，回放不恢复运动。

## 已修复并实际应用

1. **改 IP 后无法自动连接。** 旧地址不可达，当前电脑没有 `mdns-discovery` 可执行程序。原程序在回退发现时直接失败，启动依赖也没有安装它。新增随依赖安装的 Python mDNS 回退；真实广播找到了新地址，并通过原 SSH 主机密钥与序列号核对。未依赖其他项目的路径、未关闭身份校验。
2. **接收恢复丢失明确 OFF 状态。** 接收重同步或新启动会把 OFF 改成 shadow，导致重新运行推理并清空静止资格。现在保持已明确请求且反馈确认 disabled 的 OFF，同时仍重置滤波、冻结偏置、零位同步和待上力意图。此前 policy 的重启/失联保护仍退回 shadow，不恢复动作。两个新增用例先在旧逻辑失败，修复后通过；完整接收用例 13 项通过。
3. **安装之前只在电脑上的修复。** 实际安装了电脑断连后的中性输入处理、推理输入/输出及模型 SHA 记录、原生网页清除旧输入、QMI 满量程取反饱和处理。它们此前已有离线回归，本轮完成 ARM 编译、ELF 检查、OpenOCD 上传和新 MCU 启动标记核对。运动模式效果未进行实物验收。
4. **专属页面标识。** 页头新增金色高亮“星猫”，页签标题为“星猫 · XGO Duck 调试”。实际无头 Edge 检查了1280桌面与390手机宽度；无页面异常、无横向溢出，检查期间无写请求。

## 实测与仍未解决的部分

第一次安装后约30秒读取，明确 OFF 在第7.9秒后退回 shadow；300次中222次为 shadow，电机全部 disabled、禁动锁全部保留。这是上述第2项的现场线索，失败记录保留，未用后续结果覆盖。

第2项修复后再次同时读取板内与实际电脑调试台：

| 项目 | 本次结果 |
|---|---|
| 板内读取 | 300次，31.32秒，HTTP错误0 |
| 卸力与托盘禁动 | 300/300 OFF、disabled、inhibited |
| IMU及关节反馈 | 300/300 IMU有效、15轴齐全 |
| 15轴最大角度变化 | 0° |
| 静止资格 | 274/300次通过；其余在重新采集窗口 |
| 板端反馈年龄 | P99 51.99ms，最大81.09ms |
| 接收频率 | 窗口均值74.11Hz，49.17～141.86Hz，仍有批量到达/抖动 |
| 电脑调试台 | 300/300读取连接成功，HTTP错误0 |
| 电脑显示反馈年龄 | P99 352.99ms，最大426.07ms |

因此本轮确认的是连接找回、OFF恢复行为和静止禁动状态；**反馈速率与无线显示仍有抖动，不能宣称100Hz持续接收或无线问题已解决。** 两轮起点和负载不同，不能把指标直接当性能A/B。实物后倒、长期站立和走动仍未测试。

## 文件与回退

- 当前 MCU ELF SHA256：`3ebc523e713fd4398807e7a19b8934ba96c30b7c6611a5c8b066ed7dec7ec9ad`。
- 当前 controller SHA256：`a8d02835ed3b2c9a1f40f9b483f16b25cb628b3108dac0027aa9da529883a844`。
- 主控备份：应用目录 `.codex-backups/stationary-20261024/`，包含替换前源码、ELF、构建/上传日志和安装结果。第二次仅更新Host控制器，未再次烧录MCU。
- 模型、机械零位/校准JSON及禁动文件前后哈希相同。
- 本机原始证据：`pc/qa/boot-preflight-20261024.json`、`stationary-install-20261024.json`、`off-resync-install-20261024.json`、`stationary-before-off-fix-20261024.json`、`stationary-validation-20261024.json`、`starcat-ui-20261024.json`。
- 公开的精简安装摘要：`firmware-support/validation/static-20261024.json`。历史版本收据和历史验收结果保持原样。

发现流程使用 Arduino 官方 `_arduino._tcp` 服务名（[官方源码](https://github.com/arduino/mdns-discovery/blob/main/main.go)）与 [python-zeroconf API](https://python-zeroconf.readthedocs.io/en/stable/api.html)。
