# 换电脑接手调试

当前快照：2026-10-22，PC `rx-stream-r4`，基础键盘47500模型。先阅读仓库根 README；这里不要求重新烧录、校准或让机器人动作。

## 最短迁移

1. 关闭旧电脑调试台页面，避免两台同时发送控制心跳。
2. 新电脑克隆 `https://github.com/cdh66666/xgoduck-debug.git`。
3. 当前设备的直接连接配置位于 `connection/profile/`，无需口令或旧电脑ZIP。使用其他设备时先按 `connection/README.md` 修改自己的参数。
4. 在仓库根运行 `pwsh -File .\Start-Duck.ps1`；脚本会创建环境、安装依赖、直接导入配置并启动调试台，打开 `http://127.0.0.1:8791/`。需要Python 3.12/3.13和PowerShell 7。
5. 先核对同一主控、15/15舵机、IMU在线、卸力。程序重连只读，不恢复旧动作。换电脑不会修复已经存在的站立失稳。

```powershell
git clone https://github.com/cdh66666/xgoduck-debug.git
Set-Location xgoduck-debug
pwsh -File .\Start-Duck.ps1
```

设备主人已明确要求将本设备的连接配置公开随仓库提供，不再加密。导入器会设置本机SSH文件权限，不会连接或操作机器人。已有不同配置默认不覆盖；明确备份后可手动运行 `python pc/tools/import_connection.py --replace`。Linux可按根README手动建立环境，安装依赖后执行导入器与`pc/server.py`。

地址改变时可以通过Arduino mDNS重发现；已知新地址也可在启动前设置 `$env:XGODUCK_HOST='192.168.1.50'`（填你自己的局域网IP）。地址只作候选，仍会核对SSH主机密钥和主控序列号，不会把填写地址当作已经认证。

新上电默认可能是卸力推理 `shadow`。第一次测试前点“停止/卸力”，静置约3秒让OFF静止陀螺校正就绪，再明确点“站立/起身”。不要重做校准来掩盖站立问题，保留板端整个 `data/`。

## 模型与仿真

当前四个运行ONNX已随仓库提供，合计约3.30MB：基础键盘walk、官方getup、pick、roulade。当前发布控制只开放基础键盘与起身；包含一个模型文件不代表对应实机动作已经开放或验收通过。失败的旧9391前进专用模型不作默认入口。

安装可选仿真依赖：

```powershell
.\.venv\Scripts\python.exe -m pip install -r pc/requirements-simulation.txt
```

如果以后需要重新从鸭子核对并读取相同SHA的模型，运行：

```powershell
.\.venv\Scripts\python.exe pc/tools/fetch_robot_models.py --fetch
```

此工具只读四个固定文件，不发送模式、目标、上力或部署。SHA不同会拒绝覆盖；不会擅自把新的训练模型变成已部署模型。

## 给另一台电脑的Codex

可以直接粘贴：

> 请接手这个仓库。先读 README.md、docs/MOVE_TO_ANOTHER_PC.md、docs/CHANGES_20261022.md 和 MODEL_ACCEPTANCE.json。直接导入仓库connection/profile里的本设备连接配置，无需口令，启动PC调试台，先只读核对主控和卸力状态，不自动烧录、改零位、改PD或恢复旧运动。当前站立后撤和无线尖峰都没有验收通过，不要假定换电脑已解决。需要动作时按我本次明确指令执行并记录主控本地反馈。训练按 training/README.md，在独立环境/输出目录执行，不参与硬件部署。

## 固件源码

`robot-app/` 是当前应用源码；`firmware-support/` 还冻结了实际固件所需的7个库和实际编译sketch。首包修复不止一份应用头文件，不能用未打补丁的Arduino库覆盖。已有机器人无需为了换电脑重新刷机。源码核验和原生回归不能代替新环境ARM构建/上传验收。
