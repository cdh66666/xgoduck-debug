# XGO Duck 调试与训练交接（2026-10-23直接连接配置）

给另一台电脑接手的当前代码快照：完整调试台、机器人源码、固件依赖、训练代码、仿真资源和可携带的模型权重。**站立后持续后撤、实物走停和无线尖峰尚未通过验收；这不是稳定产品发布。**

仓库：https://github.com/cdh66666/xgoduck-debug

## 换电脑调试

**当前这只鸭子的连接配置已直接放入 `connection/profile/`**，无需口令、无需解密，也不用从旧电脑搬ZIP。克隆后，在仓库根运行 `pwsh -File .\Start-Duck.ps1`，脚本会安装依赖、直接导入配置并启动调试台。按设备主人明确要求，这套实例SSH身份和板端认证公开随仓库提供；其他人使用自己的鸭子时须按 `connection/README.md` 替换为自己的参数。机械零位不在连接配置里；启动只读，不自动恢复运动。

在仓库根目录运行（推荐Python 3.12/3.13）：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r pc/requirements.txt
.\.venv\Scripts\python.exe pc/tools/import_connection.py
.\.venv\Scripts\python.exe -B pc/server.py
```

打开 **http://127.0.0.1:8791/**。先看同一主控、15/15舵机、IMU在线与卸力。关闭旧电脑调试台页面，避免两台同时控制。已有鸭子不用因为换电脑重新烧录或校准。

当前基础键盘是同一47500 actor：W前进0.2、Shift+W 0.3、S后退−0.3、A/D原地转向±1，松键发零速；保留官方起身。头嘴、深Pick、自动Pick和翻滚仍未开放。当前walk SHA：`8a462e51d41fd43cbca1026698a2320f76fd84ef61d5c081cc63b7bcad7e394f`。

**姿态显示晚到不等于模型推理晚到。** 模型和传感闭环在鸭子本地；控制命令真正超过1秒中断仍会保护卸力。界面不再仅因显示迟到而强制卸力，控制回执正常时先发零速。详细差异和剩余问题见 [旧远端与当前版对比](docs/CHANGES_20261022.md)。不能保证换电脑就消除网络问题。

## 模型与可选仿真

`robot-app/python/` 包含当前四个运行ONNX，约 **3.30MB**。`training/checkpoints/` 附47500完整架构的便携PT和配对ONNX，约 **4.23MB**；新增权重合计约 **7.53MB**，不需另找父模型链。PT只剔除本机路径等`infos`元数据，Actor/Critic/Adam/迭代数逐值保留。失败旧9391前进专用模型不作默认入口。包含Pick/翻滚权重不代表对应实机动作已开放或通过。旧历史归档沿用原有内容，不改写Git历史。

```powershell
.\.venv\Scripts\python.exe -m pip install -r pc/requirements-simulation.txt
```

模型页运行的是本地MuJoCo仿真。`simulation-model/` 是交互环境资产；它与训练/实物的验收边界分别记录，不能用仿真画面代替实机通过。需要从已有鸭子重新核对相同SHA模型时，可选运行 `python pc/tools/fetch_robot_models.py --fetch`，该工具只读，不改变机器人。

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
- `reports/`、`candidates/`、`legacy/`：历史审计和候选，均有边界说明，不能与当前运行程序混装。

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
