# 第三方来源

- XGO Duck Uno Q原运行链与四个运行ONNX：[LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)。2026-10-23 默认Walk恢复为官方提交`466512e9426f61fb07d9be9b3f20df0171692150`的`57cdebf…`权重；本地47500仍在独立训练资料中。当前控制程序保留本项目的IMU、缺轴退出、头部限位和直接起身等改动，不是全栈字节原版。每份文件SHA见`SOURCE_MANIFEST.json`。
- R5仿真/显示网格源于XGO Duck项目与本用户批准的结构，原上游：[LuwuDynamics/xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl)、[xgoduck_hardware](https://github.com/LuwuDynamics/xgoduck_hardware)。源树许可证随纯源码资源保留。
- BAM纯Python源码：[rhoban/bam](https://github.com/rhoban/bam)，其`LICENSE`与README在`pc/simulation/bam_vendor/`。
- Three.js显示库及其许可证在`pc/model-assets/vendor/`；未打包原生二进制依赖。
- 模型预览缺失的 OBJ 加载器取自同一 [Three.js r179 官方源码](https://github.com/mrdoob/three.js/blob/r179/examples/jsm/loaders/OBJLoader.js)，原样保留并沿用上述 MIT 许可证。
- 当前固件的Arduino RouterBridge/RPClite、FTServo、MsgPack、ArxContainer/ArxTypeTraits、DebugLog实际依赖源码及许可证在`firmware-support/`；以源码SHA区分本地修改，不把版本字段当未改原版证明。旧目录中的重复库已从当前源码移除。
- 官方装配PDF及其中文阅读版保留原来源；R7校准固定台STL是支持现有下载入口的单个打印件，不包含整机CAD工程。

显示网格最初来自 XGO Duck RL 提交 `326d77a1122870bdefa2c36403937502c958e69c`，Apache-2.0 许可证保留于 `simulation-model/LICENSE-xgoduck_rl` 和 `pc/model-assets/LICENSE-xgoduck_rl`；Three.js r179 的 MIT 许可证在 `pc/model-assets/vendor/LICENSE`。早期板端通信参考 `xgoduck_runtime_arduino` 提交 `a6dc9f98eae6f8cc8f2bac81a97dde5a7a8bd0d2`，该上游快照没有独立 LICENSE，不以 RL 仓库的 Apache-2.0 替其授予许可。本次精简未改变第三方许可，也不声称拥有上游模型或库。

- 独立审计页使用D3.js，原库与Copyright 2010–2023 Mike Bostock许可证完整保留在`pc/model-assets/vendor/d3/`；浏览器只加载本地副本。
- 训练使用的BAM源码/许可证在`training/engine/bam/`与`training/licenses/`；RSL-RL、Torch、MuJoCo等按`training/requirements.txt`单独安装，未复制其环境或原生二进制。47500训练权重的来源、接口和元数据清理见`training/checkpoints/MANIFEST.json`。
- 无外部 Arduino 发现工具时，使用 [python-zeroconf](https://github.com/python-zeroconf/python-zeroconf)（LGPL-2.1-or-later）发现 `_arduino._tcp.local.` 广播；通过 `pc/requirements.txt` 安装，不随仓库复制依赖二进制。广播只提供地址候选，连接仍核对原有 SSH 身份。
