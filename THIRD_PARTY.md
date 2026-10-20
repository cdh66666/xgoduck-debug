# 第三方来源

- XGO Duck Uno Q原运行链、原四ONNX图：[LuwuDynamics/xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)。每份已核文件SHA见`SOURCE_MANIFEST.json`，不把本项目修改冒充上游未改版本。
- R5仿真/显示网格源于XGO Duck项目与本用户批准的结构，原上游：[LuwuDynamics/xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl)、[xgoduck_hardware](https://github.com/LuwuDynamics/xgoduck_hardware)。源树许可证随纯源码资源保留。
- BAM纯Python源码：[rhoban/bam](https://github.com/rhoban/bam)，其`LICENSE`与README在`pc/simulation/bam_vendor/`。
- Three.js显示库及其许可证在`pc/model-assets/vendor/`；未打包原生二进制依赖。
- Arduino RouterBridge/RPClite、MsgPack、ArxContainer/ArxTypeTraits、DebugLog依赖由`robot-app/sketch/sketch.yaml`声明；历史公开库源码仍保留在legacy中，不以旧库覆盖当前安装环境。
- 官方装配PDF及其中文阅读版保留原来源；R7校准固定台STL是支持现有下载入口的单个打印件，不包含整机CAD工程。

完整历史版权说明见`legacy/2026-10-17/THIRD_PARTY.md`。本发布候选未改变第三方许可，也不声称拥有上游模型或库。

- 独立审计页使用D3.js，原库与Copyright 2010–2023 Mike Bostock许可证完整保留在`pc/model-assets/vendor/d3/`；浏览器只加载本地副本。
