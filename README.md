# XGO Duck 调试与训练交接

**2026-10-26 夜间结果：** 代码修复与多项完整演员重训已留存；走停后坐起的独立仿真通过数由 29/64 提高至 61/64，但初姿能力有退步，1 m/s、整链丝滑与全部特技尚未完成。新权重未发布、未部署，实机夜间禁动。优先阅读 [结果、原速演示与未达标项](docs/OVERNIGHT_RESULT_20261026.md)，以下为历史实测和现有发布入口。

当前程序沿用官方控制结构，实机 Walk 已换为基础键盘 actor，其他官方模型保留作为待验收候选，保留调试台、校准、编号、真实模型显示、电流、动作记录回放和本地物理仿真。2026-10-24 已修复自动发现、OFF 恢复及相关离线问题，并添加“星猫”标识。最新静止调试修复了板内通知乱序、静止统计占锁排队、重复发送等待及 SSH 小包等待，取消 350 ms 显示缓冲：同条件实测乱序 98→0 次，接收 70.4→99.9 Hz，页面已知数据年龄中位数 829→98 ms。无线/系统/渲染仍有尖峰，实物后倒仍未验收，见 [根因与实测对比](docs/LATENCY_FIX_20261024.md)。此前结果保留于 [静止调试](docs/STATIC_DEBUG_20261024.md)。

随后修复了板端控制时钟和诊断争锁：同类卸力测试中，带状态轮询的实际推理由 37.01 Hz 提升到 49.80 Hz。运动意图写入单槽，在下一个本地 50 Hz 周期消费，旧意图不排队；6 秒压力测试为 50.00 Hz，板内消费等待 P99 19.71 ms。仍有调度长尾，不能保证每个周期都在 20 ms 内。已做授权地面测试，但起身后仍倒下，已恢复 OFF / 卸力 / 动作锁；完整边界见 [50 Hz 与地面实测](docs/CONTROL_CYCLE_20261024.md)。

2026-10-25 已接手 WorkBuddy 原始对话和实际构建：用户随后确认站立、前进可用，后退和转向仍会摔，整体尚未通过。保留已认可的髋 ±22°、踝 ±18°、颈/头 20°/20°，已部署托盘禁动和右腿滤波分组修复，并恢复原“星猫”调试台入口。重装腿后已完成 15/15 卸力中位校准，全部读回编码器 2048、扭矩 0、EEPROM 锁 1；23 号由约 −137.7° 回到 −0.09°。当前仍在托盘内，保持 OFF 与动作锁，不能自动开始实机动作。质量模型重建和两随机种子仿真结果见 [接手记录](docs/WORKBUDDY_HANDOFF_20261025.md)，此前实测见 [地面试验与用户反馈](docs/GROUND_STANDING_20261024.md)。

仓库：https://github.com/cdh66666/xgoduck-debug

2026-10-25 晚间：`neutral-r2` 已把“回正”与“站立姿态”分开，托盘禁动与固件指纹入口已安装，详见 [本轮修复与坐下仿真](docs/TRAY_POSES_AND_BUTT_SIT_20261025.md)。最新授权为离线全面审查和全模型重训，Shift 目标提升到 1.0 m/s；当前发布控制仍为 0.3 m/s，不能把目标值当已实现能力。训练已增加显式部署姿态/滤波基准；新权重只在通过独立评测后才准入，见 [夜间任务与参数边界](docs/OVERNIGHT_20261025.md)。坐下权重属于可选实验，不随公开包提供；缺失时正常启动调试程序并明确显示不可用。

## 换电脑调试

**当前这只鸭子的连接配置已直接放入 `connection/profile/`**，无需口令、无需解密，也不用从旧电脑搬ZIP。克隆后，在仓库根运行 `pwsh -File .\Start-Duck.ps1`，脚本会安装依赖、直接导入配置并启动调试台。按设备主人明确要求，这套实例SSH身份和板端认证公开随仓库提供；其他人使用自己的鸭子时须按 `connection/README.md` 替换为自己的参数。机械零位不在连接配置里；启动只读，不自动恢复运动。

在仓库根目录运行（推荐Python 3.12/3.13）：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r pc/requirements.txt
.\.venv\Scripts\python.exe pc/tools/import_connection.py
.\.venv\Scripts\python.exe -B pc/server.py
```

打开 **http://127.0.0.1:8791/**。先看同一主控、15/15舵机、IMU在线与卸力。关闭旧电脑调试台页面，避免两台同时控制。已有鸭子不用因为换电脑重新烧录或校准。依赖安装包含 Python mDNS 发现；没有外部 Arduino 发现程序时仍可找回 DHCP 改址后的设备，再核对固定 SSH 主机密钥及序列号。

源码的基础键盘入口使用官方 Walk：W前进0.2、Shift+W 0.3、S后退−0.3、A/D原地转向±1，松键发零速；保留官方起身。当前实机官方拆分构建的自定义键盘能力尚未全部接回原调试台。头嘴、深Pick、自动Pick和翻滚仍未通过实机验收。当前 walk SHA：`57cdebf36e7fa23116270dec96f013a6a64511f37311d322443f5297bdc3d6db`。头部32号舵机在Host与最终总线输出均限于前方±90°。托盘只允许卸力校准，不允许回中加载或运动。

当前调试台的启动、0.2前进命令、归零保持和卸力链路已完成短程实测；位移、速度、直线性与全部六档动作仍分别验收。

“实时姿态”右侧可选择历史动作，使用真实16网格模型回放，支持慢放、逐帧、拖动、显著的“返回实时”按钮与原始记录下载。完整构建可在鸭子端保存25Hz状态和已接受模式切换事件，再同步到电脑 `pc/motion-recordings/`；当前官方拆分构建尚未安装这套自动记录接口，已有档案保留。此前已导入7条历史，其中2条旧原始包缺少当时滤波姿态、指令和模型阶段，机身估算单独标识，不能作为完整策略复现。

用户最新将鸭子放入中位校准托盘，明确规定相近躺姿一律禁动；实时托盘保护独立于维护锁文件，校准完成也不自动恢复动作。页面直接显示最新实测姿态，附加缓冲为 0，过期反馈明确提示。主栏显示实测反馈频率，每 2 秒更新为整数 Hz；逐帧数据年龄保留在诊断内。此前 0.35 秒缓冲方案已替换，历史记录见 [记录与无线显示修复](docs/MOTION_HISTORY_20261023.md)，数据见 [延迟修复](docs/LATENCY_FIX_20261024.md)。

**姿态显示晚到不等于模型推理晚到。** 模型和传感闭环在鸭子本地。电脑指令超过1秒中断时清零走动、继续本地零速站立；重连不自动恢复走动。明确停止、板内传感/缺轴/零点和 MCU 看门狗仍可卸力，校准回中 hold 保留心跳超时卸力。断连处理、IMU 满量程取反和实际推理记录修复已于2026-10-24在托盘禁动条件下安装；运动中的实际效果尚未验证。原离线依据见 [站立离线修复](docs/STANDING_OFFLINE_20261023.md)，当前实测边界见 [静止调试](docs/STATIC_DEBUG_20261024.md)。

## 模型与可选仿真

`robot-app/python/` 包含四个官方运行ONNX，约 **3.18MB**。`training/checkpoints/` 仍保留47500完整架构的便携PT和配对ONNX，约 **4.23MB**，它不再是实机默认模型。训练资料独立保留；PT只剔除本机路径等`infos`元数据，Actor/Critic/Adam/迭代数逐值保留。失败旧9391前进专用模型不作默认入口。包含Pick/翻滚权重不代表对应实机动作已开放或通过。

```powershell
.\.venv\Scripts\python.exe -m pip install -r pc/requirements-simulation.txt
```

模型页运行的是本地MuJoCo仿真。`simulation-model/` 是交互环境资产；它与训练/实物的验收边界分别记录，不能用仿真画面代替实机通过。需要从已有鸭子重新核对相同SHA模型时，可选运行 `python pc/tools/fetch_robot_models.py --fetch`，该工具只读，不改变机器人。

模型库第一项“键盘控制 · 当前实机模型”使用上述同一份 Walk 权重。模型加载后直接按 Space 即可运行，再按暂停，无须先点“运行”。方向键或 WASD 控制虚拟小鸭子，也可按住画面左下角的方向按钮；Shift 配合前进快走，R 重置，Esc 清零行走指令。松键、离开页面或失去焦点会清零指令；镜头随虚拟机身移动。Pick 仿真显示前 1.6 秒张嘴、随后闭嘴的控制时序，未模拟嘴部夹持受力。此入口只控制本地仿真。验证与边界见 [键盘仿真记录](docs/KEYBOARD_SIMULATION_20261024.md)。

## 在新电脑训练

完整入口和精确命令见 **[training/README.md](training/README.md)**。使用独立环境，避免训练依赖影响调试台：

```powershell
python -m venv .venv-training
.\.venv-training\Scripts\python.exe -m pip install -r training/requirements.txt
.\.venv-training\Scripts\python.exe training/tools/check_assets.py
.\.venv-training\Scripts\python.exe training/refine.py --mode inspect --output training/outputs/47500-inspect
```

保留原生MuJoCo 3.15、XBODY机身坐标、100Hz滤波/50Hz策略、61维actor输入和14维动作、BAM物理配置。当前入口使用CPU；Windows已做短PPO和导出验证，Linux仅设计兼容，另一台电脑尚未验收。神经网络权重、物理电机配置和训练超参数是三类不同文件。

既有47500是带冻结父网络的精修结构，不能把它直接喂给全新scratch MLP的resume。已有权重的正确加载/续训与从零训练路线分别说明，不能把一次smoke说成已经训练好。训练不连接硬件；新训练结果需独立仿真评估后再单独部署。

## 源码布局

- `pc/`：已认可的完整界面、非模态控制、校准、编号、电流曲线/CSV、无线状态流和本地仿真。
- `robot-app/`：当前应用源码和4个运行模型；不含实际 `data/`、认证或机械零位。
- `firmware-support/`：已安装固件实际使用的7个依赖库、11份编译sketch、源码锁定及首包/启动原生回归。**只复制应用头文件不足以复现固件**，构建时按其README保留库补丁。
- `training/`：独立训练、环境资源、导出、评估和训练权重交接。
- `reports/`：保留历史分析与证据摘要。旧控制程序及未准入候选不再混在当前源码中，可由精简前快照恢复。

`RELEASE_RUNTIME.json` 记录当前代码与模型SHA；`SOURCE_MANIFEST.json` 区分原始与便携适配SHA，`PACKAGE_FILES.json` 是公开文件库存。便携代码使用相对路径、当前解释器和用户自己的身份配置，不能把本机路径/部署回执当作新电脑依赖。

## 离线检查

完整`tests/`包含本地仿真回归，先安装`pc/requirements-simulation.txt`；仅运行调试台不需要仿真依赖。训练检查使用独立训练环境。

```powershell
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -p "test_*.py"
node --test tests/test_control_telemetry_gap.mjs tests/test_official_workbench_pose.mjs
.\.venv-training\Scripts\python.exe -m unittest discover -s training/tests -v
python firmware-support/verify_sources.py
```

这些检查不应访问设备或启动动作。软件、仿真、实物验收分别记录在 `SNAPSHOT_VERIFICATION.json`、`training/VALIDATION.json`、`MODEL_ACCEPTANCE.json`。历史2026-10-20说明保存在 [原快照README](reports/20261020-snapshot-README.md)。第三方来源与许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
