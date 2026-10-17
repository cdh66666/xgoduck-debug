# XGO Duck 调试

轻量的 XGO Duck / Arduino Uno Q 调试台：15 轴反馈、慢速回正与停止卸力、零位记录、实时姿态，以及备用 USB 编号工具。电脑端是 Python HTTP 服务和原生 JavaScript，不需要 Node.js 构建。

## 启动

需要 Windows、Python 3.10+、OpenSSH；自动寻找主控地址还需 Arduino 的 `mdns-discovery`。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe server.py
```

打开 <http://127.0.0.1:8791/>。未配置设备也能启动并浏览模型，连接状态会如实显示离线。启动和重连不会发出运动指令。

## 连接自己的设备

1. 将 `config/device.example.json` 复制为根目录 `device.local.json`，填写自己的 Uno Q 序列号。
2. 将 `config/ssh-identity.example.json` 和 `config/wireless-calibration-auth.example.json` 复制到 `tools/.private/`，去掉文件名中的 `.example`。填写同一序列号，把自己的 SSH 私钥和经过核实的 `known_hosts` 放在该私有目录；认证密钥必须与板端 `data/calibration_key` 一致。
3. 运行 `python tools/discover_wireless_unoq.py`，或在网页的通信工具中连接。自动发现找不到工具时，可用环境变量 `XGODUCK_MDNS` 指定 `mdns-discovery.exe`。

现有机器的零位与认证数据保留在本机和板端，不在仓库中。不要把示例内容覆盖到已调好的机器。`XGODUCK_SERIAL` 可覆盖本地配置中的序列号。

`audited_turn_targets` 默认空，禁止复用别人的跨圈坐标；只有某轴已完成实测核对并保留对应板端审计记录时，才在自己的私有配置中填写该轴的两个等价编码值。其他速度、反馈、跟踪与卸力保护保持原有契约。

## 代码目录

- 根目录：电脑端 HTTP、无线连接与状态转发。
- `assets/`、`model-assets/`：网页和当前姿态模型；模型是显示参考，不是实机位置证明。
- `board/`：HOME ABI5 的 Uno Q 应用、MCU 源码及必要的固件库。
- `tools/`：只读发现与发布同步工具。

板端需要 Arduino App Lab / Uno Q 平台 `arduino:zephyr 1.0.0`。`board/sketch/sketch.yaml` 固定库路径；修改过的 Bridge/RPClite 库一并保留。电脑端无需安装这些固件依赖。板端设备配置放在 `board/python/device.local.json`；已有部署升级应保留整个 `data/` 目录。此仓库不附带针对某台机器的自动刷写包。

## 回正验证边界

HOME5 已在本项目设备安装，包含从当前位置向已保存零位逐轴慢回、到位保持与停止卸力保护。实际回正、保持和停止效果仍以对应设备的现场验收为准。公开版仅将设备身份改为配置输入；不能把另一台机器的零位或跨圈参考直接搬过来。

发布内容不包含训练模型、CAD 工程、工具链、历史候选包、调试日志、私钥或真实零位。源代码与发布调整记录见 `SOURCE_MANIFEST.json`。需要从原工作目录更新时：

```powershell
python tools/sync_source.py --source <servo-debug目录> --approved-update <已批准增量清单.json>
```

该命令按清单复制所需文件并做少量便携化处理，不读取或复制私钥内容，不操作硬件。

上游模型、组装说明与第三方库见 [THIRD_PARTY.md](THIRD_PARTY.md)。
