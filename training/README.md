# 换电脑使用47500模型、继续训练或从零训练

## 2026-10-25 修正物理参数后的新训练

旧稳定版的结论只适用于旧模型和旧 HOME。新训练使用 `model/robot_inertial_candidate_20261025.xml`（826 g；质量来源和未实测惯量范围见同名 JSON），并显式传入 `--runtime-profile calibrated-20261025`：髋 ±22°、踝 ±18°、头部滤波 0.2，右髋归腿部滤波。这个选项同时约束观测零点、目标零点、仿真起始姿态和 ONNX 元数据。旧 `legacy` 保留以复现旧结果，不能与新基准的 checkpoint 混续。

```powershell
.\.venv-training\Scripts\python.exe -B training/full_refine.py --mode train --iterations 6000 --max-wall-seconds 1800 --workers 6 --envs-per-worker 4 --curriculum speed --max-forward 0.3 --runtime-profile calibrated-20261025 --model-xml training/model/robot_inertial_candidate_20261025.xml --trace-every 100 --output training/outputs/my-calibrated-run
.\.venv-training\Scripts\python.exe -B training/evaluate.py --checkpoint training/outputs/my-calibrated-run/checkpoint.pt --seeds 8 --max-forward 0.3 --output training/outputs/my-calibrated-run/evaluation.json
```

`speed` 保留零速、0.2 m/s、后退和原地转向；仅把 Shift 前进分支扩展为 `--max-forward` 的 70%–100%（下限 0.2）。上限可逐级到 1.0 m/s。普通六点验收默认仍是旧 0.3，评测高速候选必须明确指定上限。50 Hz 为物理时间中的实际策略调用计数；CPU 仿真可以快于实时，不能由此宣称实机每周期必达 20 ms。

完整 actor 的 226702 个参数均参与更新；旧权重只作初始化，固定标准化统计和探索噪声。这不同于只更新残差，也不同于从随机权重训练。独立评测未通过的候选不准入实机或公开运行权重。

坐下工具 `train_butt_sit.py` 使用公开的派生站姿参考 `model/standing_seed_20261025.json`，无需本机私有原始记录。`search_butt_raise.py` 搜索坐下后连续起身的闭环教师轨迹，结果不是神经模型，也不代表已验收。`render_butt_sit.py --output <训练目录> --ffmpeg <可执行文件>` 渲染真实接触轨迹，`--before-model` 可选提供旧模型对照，`--font` 指定中文字体。完整夜间范围见 [任务记录](../docs/OVERNIGHT_20261025.md)。

这里包含训练代码、R5 无 EVA 仿真 XML/实际网格、BAM 执行器配置、PPO 配置，以及 **47500 的便携 PT（3.31MB）和同版 ONNX（0.918MB）**。只打包 `checkpoints/MANIFEST.json` 白名单里的两份权重；不含设备密钥、校准数据、本机环境或日志。训练入口不连接机器人。

2026-10-23本机已建立独立环境。早期5轮实训见 [SESSION_20261023.md](SESSION_20261023.md) 和 [VALIDATION_20261023.json](VALIDATION_20261023.json)；后续站立/走停训练及原生录像见 [SESSION_20261023_STABILITY.md](SESSION_20261023_STABILITY.md)。站立候选在独立8-seed集的静止倾角RMS减少约49%，48/48基础矩阵通过，但走停尚未整体改进，继续保留原运行基线。**均未操作、部署或通过实机验收。** 新候选和逐时刻数据保存在本机独立outputs目录，未加入公开权重白名单。

用户最新要求先给稳定、不会往后倒的版本：已将既有r4-1200固定为本地**仿真稳定基线v1**，保存于`outputs/stable-backward-20261023-2249/stable-v1`。新8-seed长时检查32/32通过，累计44分钟、96次后退走停和48次3N向后推力，无倾倒、恢复或片内重置；另有48秒连续双视角原生录像。这个结果限定在已测仿真范围，未操作实机，也不宣称总体制动优于原版。

有五条独立路线：`refine.py` 继续47500的原后退精修；`full_refine.py` 学习完整47500 Actor（含零速base）；`standstop.py` 冻结原Actor，学习零速站立和停止修正；`brake.py` 冻结已验证的站立候选，只学习动态制动；`train.py` 从随机初始化的 `scratch MLP + PPO` 学起。47500的base、residual、feature head、back_skip及normalizer都包含在最终PT里，不需要父模型链。各入口检查checkpoint类型并拒绝混用。`reference-refinement/` 保留旧精修源码，实际便携加载器是 `refine_model.py`。

## 1. 安装与最短自检

以下在仓库根目录执行。2026-10-22迁移验证使用Windows Python3.13；2026-10-23本机独立环境使用Python3.12.14。代码使用 pathlib 和 multiprocessing spawn，设计上可用于 Linux，但未在 Linux 上安装或运行验证。Linux 使用同版 Python，并将下方解释器路径替换为 `.venv-training/bin/python`。

```powershell
python -m venv .venv-training
.\.venv-training\Scripts\python.exe -m pip install --upgrade pip
.\.venv-training\Scripts\python.exe -m pip install -r training/requirements.txt
.\.venv-training\Scripts\python.exe training/tools/check_assets.py
.\.venv-training\Scripts\python.exe -m unittest discover -s training/tests -v
.\.venv-training\Scripts\python.exe training/train.py --mode smoke --iterations 1 --workers 1 --envs-per-worker 2 --output training/outputs/smoke
```

输出目录必须是新目录，避免覆盖旧训练。自检会实际进行 24 个控制时刻 × 2 环境、一次 PPO 更新，导出本机生成的 `policy.onnx`，比较 64 个 Torch/ONNX 样本（每个样本输入61维、输出14维）。它证明入口可运行、参数在学习、导出一致，**不证明新训练策略会站稳或走路**。

依赖中 MuJoCo 固定 3.15.0、RSL-RL 固定 5.0.1；NumPy/Torch/ONNX 允许指定范围内兼容版本，因此应先跑上述自检。没有附带 CUDA、venv 或原生 DLL。本入口始终使用 CPU 物理和 CPU 学习，即使 Torch 是 CUDA 安装版也不占 GPU。

## 2. 加载既有47500，或继续原后退精修

```powershell
.\.venv-training\Scripts\python.exe training/refine.py --mode inspect --output training/outputs/47500-inspect
.\.venv-training\Scripts\python.exe training/refine.py --mode smoke --iterations 1 --workers 1 --envs-per-worker 2 --output training/outputs/47500-smoke
.\.venv-training\Scripts\python.exe training/refine.py --mode train --iterations 3000 --workers 4 --envs-per-worker 2 --output training/outputs/47500-continue
```

`inspect` 严格加载完整Actor/Critic和Adam10状态，再用64个61维输入与原版47500 ONNX比较，不训练。`smoke`在此基础上只做1次PPO更新，确认非back_skip的Actor张量、normalizer和固定sigma保持不变。

这条续训保持原47500职责：**只有868个back_skip参数和Critic64可学习，原base/residual/head冻结**，不等于把前进、头部或1.2m/s扩展一起重新训练。环境/RNG重新初始化，不声称物理环境逐帧续接或新训练已收敛。续后续本入口产物时加 `--checkpoint training/outputs/47500-continue/checkpoint.pt`。

这条`refine`路线不会自动改善零速站立，因为后退修正层在零速时不参与输出。站立训练可用下方完整Actor或站停入口；不能把“继续后退精修”当作当前实物后撤问题已修复。

完整47500路线使用独立入口和 `xgoduck-full-47500-bodyframe-v2` 类型：

```powershell
.\.venv-training\Scripts\python.exe training/full_refine.py --mode train --curriculum six --seed 20261025 --iterations 300 --max-wall-seconds 300 --workers 4 --envs-per-worker 4 --output training/outputs/full47500
.\.venv-training\Scripts\python.exe training/evaluate.py --checkpoint training/outputs/full47500/checkpoint.pt --seeds 8 --output training/outputs/full47500/evaluation.json
```

首次训练逐值恢复原Actor/Critic，更新前与原ONNX比较；原来仅10个参数对象的backskip Adam不适用完整Actor，因此**明确重新建立Adam，自己的迭代数从0起计**，不冒称原优化器的精确续接。全部226702个base/residual/head/back_skip参数可学习，原sigma与actor normalizer冻结并在结束时核对不变。默认混合六点课程中也有零速及运动后切零速；站立base确实有梯度，区别于后退层的零速门控。默认固定学习率1e-7；1e-6实验曾损坏后退偏航精度，保留在本机记录中。后续只用本入口产物 `--resume .../checkpoint.pt`，显式传该轮保存的 `--config .../config.json`，仍须完整矩阵验证，不能凭精确初始加载认定训练后保留了步态。

便携PT只删去原`infos`里本机路径/会话等非训练元数据，Actor、Critic和小型Adam状态均逐值保留，所以文件SHA与原PT不同；来源SHA和当前SHA/大小都在 `checkpoints/MANIFEST.json`。ONNX保持原字节，SHA为 `8a462e51d41fd43cbca1026698a2320f76fd84ef61d5c081cc63b7bcad7e394f`。权重小计约4.23MB，没有再上传4.84MB的父checkpoint。

针对站立与走停的入口：

```powershell
.\.venv-training\Scripts\python.exe training/standstop.py --mode train --curriculum stability --seed 20261081 --iterations 2000 --max-wall-seconds 600 --workers 8 --envs-per-worker 2 --output training/outputs/standstop
.\.venv-training\Scripts\python.exe training/tools/evaluate_stability.py --policy training/outputs/standstop/policy.onnx --seeds 8 --first-seed 20261220 --push-n 2 --output training/outputs/standstop/stability.json
.\.venv-training\Scripts\python.exe training/evaluate.py --checkpoint training/outputs/standstop/checkpoint.pt --seeds 8 --output training/outputs/standstop/basic-matrix.json
```

`standstop`精确恢复并冻结原Actor权重与normalizer，新增零初始化的61→64→14修正层，只有零命令时参与，幅度限制为±0.10rad。探索sigma按config设置后固定、不学习；当前默认0.005、学习率1e-5，用于减小精细站立训练的噪声，实际推理不采样。相同61维输入和非零命令下，输出与原版逐值一致；停止改变了后续物理状态，连续走停仍须独立验证。课程交替运动与零速，每段3–5秒；部分站立段加入0.8–2N、0.16秒本地仿真推力。训练环境、奖励权重保存在`run.json`与checkpoint的`training_environment`。这是新的候选训练路线，不能据此宣称比原版稳定。当前实验及录像见 [SESSION_20261023_STABILITY.md](SESSION_20261023_STABILITY.md)。

`--curriculum stop`提高停止样本比例：运动3–5秒后零速1.3–1.8秒，25%的零速段延长为4–5秒以保留站立练习，并施加2N四向随机推力0.16秒；80%的episode从运动开始。它使用相同v3零速奖励：原密集奖励上叠加速度/偏航高斯奖励、窄倾角奖励、实际路径惩罚和连续倾角惩罚。恢复自己的checkpoint仍须传保存的config，保留被认可的快照，不把更多更新当作改进。

`brake.py --initial-checkpoint <已验证的standstop.pt>`精确加载并冻结已有站立修正，新增只有零速动态状态才参与的4878参数制动层，使用全新Adam。安静状态保留站立输出，非零命令保留原版输出；仍需逐场景验证状态轨迹。该类型为`xgoduck-brake-47500-bodyframe-v2`，结束checkpoint包含全部原版、站立和制动权重，独立推理无需父checkpoint；首次初始化需要显式提供站立源，后续用自己的config和`--resume`恢复。短程自检应至少20更新，保证经历一次运动→停止，而不是只检验前两次还在行走的更新。当前仍在训练，未证明走停明显改善。

## 3. 可选：从随机权重先学站立，再学习六个控制点

```powershell
.\.venv-training\Scripts\python.exe training/train.py --mode train --curriculum stand --iterations 8000 --workers 4 --envs-per-worker 2 --output training/outputs/stand
.\.venv-training\Scripts\python.exe training/train.py --mode train --curriculum six --iterations 8000 --workers 4 --envs-per-worker 2 --resume training/outputs/stand/checkpoint.pt --output training/outputs/six
```

8000 是显式训练预算示例，不是保证足够。根据 `metrics.jsonl` 的失败、真实速度与独立评估决定是否继续，不能仅凭 reward 判定成功。线程较少或内存不足的电脑可减为 `--workers 1 --envs-per-worker 2`。恢复只接本入口产生、配置完全相同的 checkpoint；不读取本机旧 parent 权重、ONNX 或其他训练目录。

输出：`checkpoint.pt`（当前 actor/critic/optimizer，供本机续训）、`policy.onnx`、`metrics.jsonl` 和 `run.json`。新训练的权重与输出目录受 gitignore 排除，不自动上传 GitHub；仓库只包含上述明确白名单的47500权重。

可加 `--max-wall-seconds 600` 为一轮训练设置时间上限；当前 PPO 更新结束后保存、导出并退出，不强制截断一次更新。`run.json` 记录实际更新数、结束原因和采样数。每100次更新另存 `checkpoint_XXXXXX.pt`，`config.json` 固定本轮超参数，`rollout.jsonl.gz` 保留每个控制时刻的各环境动作、命令、速度、倾角、高度、奖励、终止与重置计数。`metrics.jsonl` 同时记录失败原因和耗时。恢复仍重新初始化环境和 RNG，不宣称逐帧续接。

`--config <JSON路径>` 可在独立配置中调整 PPO 采样、学习率和探索分布；物理、维数、时序、HOME 等非 PPO 字段必须与默认配置完全一致，actor/critic 类型必须符合各入口。恢复自定义配置的 checkpoint 时也要传入该轮保存的 `config.json`；配置仍按完整相等校验。独立评估使用 checkpoint 内保存的 PPO 配置，并重新核对固定契约，不依赖后续修改的默认超参数。

## 4. 独立验收（兼容上述五类checkpoint）

```powershell
.\.venv-training\Scripts\python.exe training/evaluate.py --checkpoint training/outputs/six/checkpoint.pt --seeds 8 --output training/outputs/six/evaluation.json
```

矩阵包含每 seed 的零速站立 15 秒，以及前进 0.2、快走 0.3、后退 −0.3 m/s、左右原地转向 ±1 rad/s 各运动 12 秒后零速停止 6 秒。评估不允许自动 reset 或起身恢复；倾倒、恢复切换和终止会保留为失败。原生训练环境的 3 秒低进度停滞保护也保留为提前失败，不让 reset 掩盖它。运动主轴要求目标速度的 80%–120%、线速度 RMSE ≤0.05 m/s、角速度 RMSE ≤0.2 rad/s；停止窗口要求线速度 ≤0.03 m/s、角速度 ≤0.08 rad/s、倾角 ≤25°。报告保留逐时刻数据，不把平均值覆盖失败。

这是可移植的独立基础矩阵，**还不包含原 47500 最终报告的 126 秒连续切换、扰动、起身、头身混合、嘴部或翻滚验收**。不能把 `PASS_THIS_MATRIX` 扩写成完整离线验收或实机通过；长时间切换、扰动与实物验证需另外完成。

对照两次评估可运行（绘图工具额外需要 `matplotlib`）：

```powershell
.\.venv-training\Scripts\python.exe training/tools/compare_evaluations.py --evaluation 47500 training/outputs/baseline.json --evaluation candidate training/outputs/six/evaluation.json --output training/outputs/comparison
```

输出 `comparison.json` 和 `comparison.png`；图中保留每个 seed 的轨迹与失败终点，不用平均曲线隐藏失败。通过计数仍由完整三轴、倾角、时长与停止标准共同决定。

`tools/evaluate_stability.py`另测停止后第一秒的平面路径、偏航路径、倾角RMS和连续0.5秒满足速度阈值的停稳时间，以及20秒连续走停和四向推力；每个场景片内禁止reset/recovery。`tools/compare_stability.py`按相同seed、场景与力配对。`tools/render_native_video.py --left 原版.onnx --right 候选.onnx --output 新视频.mp4`渲染同条件并排原生物理录像，默认48秒、25fps、1倍速度，每段独立初始化并明确标注。FFmpeg须在PATH中，绘制网格和颜色仅影响显示。单看本基础矩阵几秒后的停止窗口，不能判断松键瞬间是否改善。

## 保留的模型契约

| 链路 | 当前内容 |
|---|---|
| Actor | 61维输入→14维原始动作；scratch随机初始化；其余入口精确加载便携47500；standstop另加零命令修正层 |
| Critic | 64 维，仅训练时增加真实机身坐标 vx/vy/vz |
| 时间 | MuJoCo 200Hz、传感器 IIR 100Hz、策略 50Hz |
| 坐标 | 机身 `XBODY/local=1`，不是惯性坐标 `BODY` |
| 关节目标 | 原始动作历史 + 0.45 IIR + HOME；20ms 目标队列；BAM P6、7.4V |
| 场景 | 与冻结基础模型相同 XML 和网格、13 处显示网格地面接触扩展 |
| 基础控制 | 零速、W 0.2、Shift+W 0.3、S −0.3、A/D ±1；头/身体输入全零 |
| 已知限制 | BAM 没有已辨识的飞特 D20 映射；0.8255kg 是拆 EVA 前整机称量；COM/惯量仍有不确定性 |

`params/1910_m6.json` 是运行仿真必须的电机物理配置，`config.json` 是训练超参数，均不是神经网络权重。嘴 ID34 不属于 14 维 actor。训练终止/重新采样属于训练流程，不能拿训练中带 reset 的统计代替不重置验收。

来源与修改边界见 `SOURCE_LINEAGE.json`。BAM 为 Apache-2.0，许可证在 `licenses/`；其余上游来源沿用仓库根 `THIRD_PARTY.md`。2026-10-22迁移检查验证了随机scratch短训练、47500的精确加载/ONNX对齐和1次继续PPO，当时未启动长训练。2026-10-23本机实训结果见上方独立记录。47500与当前候选的基础原生CPU矩阵通过不代表实物站立通过。
