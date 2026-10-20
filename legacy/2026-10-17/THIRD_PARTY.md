# 上游与第三方

- 显示模型来自 [LuwuDynamics/xgoduck_rl](https://github.com/LuwuDynamics/xgoduck_rl/tree/326d77a1122870bdefa2c36403937502c958e69c) 的网格和 MJCF，保留 [Apache-2.0 许可](model-assets/LICENSE-xgoduck_rl)。本项目增加了嘴部拆分等可视化调整；参考模型仅用于调试显示。
- [官方组装 PDF](https://github.com/LuwuDynamics/xgoduck_hardware/blob/90ec55634d700c88fd7aa9281c282be30f56e7e0/Assembly_Guide.pdf) 仅外链，不在本仓库重复打包。
- `model-assets/vendor/` 为 [Three.js r179](https://github.com/mrdoob/three.js/tree/r179)，保留对应 [MIT 许可](model-assets/vendor/LICENSE)。
- `board/firmware_dependencies/libraries/` 保留 Arduino_RouterBridge、Arduino_RPClite、MsgPack、ArxContainer、ArxTypeTraits 与 DebugLog 的原许可；其中 Bridge/RPClite 包含本项目的通信修正。
- 原工作区训练项目与本仓库分开维护；本仓库不包含 ONNX、训练权重或训练依赖。
- 板端通信基础参考 [xgoduck_runtime_arduino](https://github.com/LuwuDynamics/xgoduck_runtime_arduino/tree/a6dc9f98eae6f8cc8f2bac81a97dde5a7a8bd0d2)。该上游快照未包含独立 LICENSE，RL 仓库的 Apache-2.0 不自动覆盖此部分；本仓库不为其另行授予许可证。
