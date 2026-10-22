# 第三方来源

- XGO Duck Uno Q原运行链和getup/Pick/roulade ONNX：[LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)。当前walk为本地47500训练导出，不能称原版官方walk；每份已核文件SHA见`SOURCE_MANIFEST.json`。
- R5仿真/显示网格源于XGO Duck项目与本用户批准的结构，原上游：[LuwuDynamics/xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl)、[xgoduck_hardware](https://github.com/LuwuDynamics/xgoduck_hardware)。源树许可证随纯源码资源保留。
- BAM纯Python源码：[rhoban/bam](https://github.com/rhoban/bam)，其`LICENSE`与README在`pc/simulation/bam_vendor/`。
- Three.js显示库及其许可证在`pc/model-assets/vendor/`；未打包原生二进制依赖。
- 当前固件的Arduino RouterBridge/RPClite、FTServo、MsgPack、ArxContainer/ArxTypeTraits、DebugLog实际依赖源码及许可证在`firmware-support/`；以源码SHA区分本地修改，不把版本字段当未改原版证明。历史库仍在legacy中，不能覆盖当前构建。
- 官方装配PDF及其中文阅读版保留原来源；R7校准固定台STL是支持现有下载入口的单个打印件，不包含整机CAD工程。

完整历史版权说明见`legacy/2026-10-17/THIRD_PARTY.md`。本发布候选未改变第三方许可，也不声称拥有上游模型或库。

- 独立审计页使用D3.js，原库与Copyright 2010–2023 Mike Bostock许可证完整保留在`pc/model-assets/vendor/d3/`；浏览器只加载本地副本。
- 训练使用的BAM源码/许可证在`training/engine/bam/`与`training/licenses/`；RSL-RL、Torch、MuJoCo等按`training/requirements.txt`单独安装，未复制其环境或原生二进制。47500训练权重的来源、接口和元数据清理见`training/checkpoints/MANIFEST.json`。
