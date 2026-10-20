# XGO Duck 当前代码快照（2026-10-20）

这是当前调试台和 Uno Q 程序的源码快照，**不是稳定发布，站立稳定性与受推恢复均为 NOT_PASSED**。用户实机观察到站起后向后撤步；当前交互仿真的受推恢复也未通过。尚未确定单一根因，不能把页面流畅、源码一致或离线检查当作机器人验收通过。

默认配置仅允许原版零速站立 / 必要起身与普通头嘴控制。行走、快跑、深度 Pick、自动 Pick 和翻滚默认停用；历史翻滚曾触发电池供电中断，原因未解决。失败的 9391 前进图及新 600 轮候选不包含在发布运行清单中。禁用入口是当前范围限制，不是策略或供电问题已经修复。

## 内容与证据边界

- `pc/`：当前完整调试台、无线状态流、舵机电流曲线/CSV、校准流水、编号及可选交互仿真。控制为非模态侧面板，提示为悬浮通知，顶部六张状态卡保留。
- `robot-app/`：由当前37份绑定源导出，除失败的前进 ONNX 外保留36份默认源。包括当前校准中位 HOME、Host、MCU、原四个 ONNX 及发布范围 guard。原始 SHA 与公开 SHA 在 `SOURCE_MANIFEST.json` 对照。
- `pc/assets/calibration-tray-r7/` 与 `pc/downloads/`：同源 R7 托盘和鸭子真实 STL 预览、manifest 与打印 STL。仅同步已有几何，不代表新增装配/打印验收。
- `simulation-model/`：当前 R5 无 EVA XML、网格与 BAM 参数；交互环境包含显示碰撞扩展，不能视为训练环境或实机的精确复制。
- `legacy/2026-10-17/`：原公开 master 完整归档供追溯，不能与当前程序混装。

校准入口先展示 R7 摆放预览，用户确认后才执行一次 `recalibrate`：串行停止/卸力、进入、读回并逐轴保存。不会因打开页面、重连或打开预览而写校准。当前失败、取消、缺轴与历史记录均保留；旧15颗回执不会冒充本次完成。校准覆盖 EEPROM 零位，请先正确放入固定托盘。

HOME 回到已保存的校准中位并保持，不进入旧默认站姿。15轴新鲜反馈已在中位时只提示，不重新发动作。固定托盘内只做校准，站立/起身应取出后在地面明确启动。

电流是15颗舵机反馈电流幅值之和，**不是电源端总电流**，不能直接推算电池功率或证明断电原因。原始记录、CSV、真实零位、密钥、SSH配置、设备序列号、连接缓存和原始轨迹均未上传。

## 电脑启动

Windows、Python 3.10+、OpenSSH；网页无需 Node 构建。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r pc/requirements.txt
.\.venv\Scripts\python.exe -B pc/server.py
```

入口 `http://127.0.0.1:8791/`。无自己的身份和认证配置时只显示离线，不借用开发机器的配置。

1. 将 `pc/config/device.example.json` 复制为 `pc/device.local.json`，填写自己的 Uno Q 序列号，也可设置 `XGODUCK_SERIAL`。
2. 在 `pc/tools/.private/` 放自己的固定 SSH 身份、已核实 known_hosts 和与板端相同的认证密钥，模板见 `pc/config/*.example.json`。实际文件受 gitignore 排除。
3. 将 Arduino `mdns-discovery` 放入 PATH 或设置 `XGODUCK_MDNS`。自动发现与重连只读，不恢复旧运动目标。

**保留原主控整个 `data/`、认证和校准备份。** 仓库不提供设备零位，不附自动烧录脚本；旧 legacy 更新器不能用于本版。便携化适配包括相对路径清单、用户身份配置、当前 Python 解释器和移除私有部署回执依赖，未独立安装到另一台机器人验收。

## 可选仿真

```powershell
.\.venv\Scripts\python.exe -m pip install -r pc/requirements-simulation.txt
```

在启动PC服务的同一Python环境安装这些依赖；也可设置 `XGODUCK_SIM_PYTHON` 指向自己已配置好的仿真解释器。原工作环境的原生vendor与Python ABI绑定，不随包复制。

依赖锁定 MuJoCo 3.15.0、ONNX Runtime 1.26.0，使用兼容 SciPy 1.15.3 的 NumPy 2.2.6；不打包原生二进制、venv 或 CUDA。模型库打开预览才运行独立 CPU 仿真，加载包内 XML、网格和同源策略。无设备的离线启动/API 与资源绑定有检查；本快照不宣称稳定站立、受推恢复、速度跟踪或松键停稳已经通过。

## 离线检查

```powershell
python -B -m unittest discover -s tests -p "test_*.py"
node tests/test_workbench.mjs
node tests/test_calibration_flow.mjs
node tests/test_home_middle.mjs
node tests/test_current_curve_ui.mjs
```

检查使用假隧道/合成数据，禁止连接硬件。`SOURCE_MANIFEST.json` 记录原始与便携化 SHA，`PACKAGE_FILES.json` 记录每个公开文件；`MODEL_ACCEPTANCE.json` 明确当前失败与未验证项。源码快照发布与实机通过是两回事。

第三方来源和许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
