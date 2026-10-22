# 换电脑使用47500模型、继续训练或从零训练

这里包含训练代码、R5 无 EVA 仿真 XML/实际网格、BAM 执行器配置、PPO 配置，以及 **47500 的便携 PT（3.31MB）和同版 ONNX（0.918MB）**。只打包 `checkpoints/MANIFEST.json` 白名单里的两份权重；不含设备密钥、校准数据、本机环境或日志。训练入口不连接机器人。

有两条明确分开的路线：`refine.py` 加载47500的复合网络继续原后退精修；`train.py` 采用随机初始化的完整 `scratch MLP + PPO` 从零训练。47500的base、residual、feature head、back_skip及normalizer都已经包含在最终PT里，不需要另下载父模型链。不能把47500直接交给scratch的`--resume`；两种入口会检查类型并拒绝混用。`reference-refinement/` 保留旧精修源码供研究，实际便携加载器是 `refine_model.py`。

## 1. 安装与最短自检

以下在仓库根目录执行。本轮实测 Windows Python 3.13；代码使用 pathlib 和 multiprocessing spawn，设计上可用于 Linux，但本次未在 Linux 上安装或运行验证。Linux 使用同版 Python，并将下方解释器路径替换为 `.venv-training/bin/python`。

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

这条`refine`路线不会自动改善零速站立，因为后退修正层在零速时不参与输出。若要重新学习站立，可先用第3节的完整scratch actor；不能把“继续后退精修”当作当前实物后撤问题已修复。

便携PT只删去原`infos`里本机路径/会话等非训练元数据，Actor、Critic和小型Adam状态均逐值保留，所以文件SHA与原PT不同；来源SHA和当前SHA/大小都在 `checkpoints/MANIFEST.json`。ONNX保持原字节，SHA为 `8a462e51d41fd43cbca1026698a2320f76fd84ef61d5c081cc63b7bcad7e394f`。权重小计约4.23MB，没有再上传4.84MB的父checkpoint。

## 3. 可选：从随机权重先学站立，再学习六个控制点

```powershell
.\.venv-training\Scripts\python.exe training/train.py --mode train --curriculum stand --iterations 8000 --workers 4 --envs-per-worker 2 --output training/outputs/stand
.\.venv-training\Scripts\python.exe training/train.py --mode train --curriculum six --iterations 8000 --workers 4 --envs-per-worker 2 --resume training/outputs/stand/checkpoint.pt --output training/outputs/six
```

8000 是显式训练预算示例，不是保证足够。根据 `metrics.jsonl` 的失败、真实速度与独立评估决定是否继续，不能仅凭 reward 判定成功。线程较少或内存不足的电脑可减为 `--workers 1 --envs-per-worker 2`。恢复只接本入口产生、配置完全相同的 checkpoint；不读取本机旧 parent 权重、ONNX 或其他训练目录。

输出：`checkpoint.pt`（当前 actor/critic/optimizer，供本机续训）、`policy.onnx`、`metrics.jsonl` 和 `run.json`。新训练的权重与输出目录受 gitignore 排除，不自动上传 GitHub；仓库只包含上述明确白名单的47500权重。

## 4. 独立验收（兼容两类checkpoint）

```powershell
.\.venv-training\Scripts\python.exe training/evaluate.py --checkpoint training/outputs/six/checkpoint.pt --seeds 8 --output training/outputs/six/evaluation.json
```

矩阵包含每 seed 的零速站立 15 秒，以及前进 0.2、快走 0.3、后退 −0.3 m/s、左右原地转向 ±1 rad/s 各运动 12 秒后零速停止 6 秒。评估不允许自动 reset 或起身恢复；倾倒、恢复切换和终止会保留为失败。原生训练环境的 3 秒低进度停滞保护也保留为提前失败，不让 reset 掩盖它。运动主轴要求目标速度的 80%–120%、线速度 RMSE ≤0.05 m/s、角速度 RMSE ≤0.2 rad/s；停止窗口要求线速度 ≤0.03 m/s、角速度 ≤0.08 rad/s、倾角 ≤25°。报告保留逐时刻数据，不把平均值覆盖失败。

这是可移植的独立基础矩阵，**还不包含原 47500 最终报告的 126 秒连续切换、扰动、起身、头身混合、嘴部或翻滚验收**。不能把 `PASS_THIS_MATRIX` 扩写成完整离线验收或实机通过；长时间切换、扰动与实物验证需另外完成。

## 保留的模型契约

| 链路 | 当前内容 |
|---|---|
| Actor | 61 维输入 → 14 维原始动作；MLP 512/256/128；随机初始化、无教师 |
| Critic | 64 维，仅训练时增加真实机身坐标 vx/vy/vz |
| 时间 | MuJoCo 200Hz、传感器 IIR 100Hz、策略 50Hz |
| 坐标 | 机身 `XBODY/local=1`，不是惯性坐标 `BODY` |
| 关节目标 | 原始动作历史 + 0.45 IIR + HOME；20ms 目标队列；BAM P6、7.4V |
| 场景 | 与冻结基础模型相同 XML 和网格、13 处显示网格地面接触扩展 |
| 基础控制 | 零速、W 0.2、Shift+W 0.3、S −0.3、A/D ±1；头/身体输入全零 |
| 已知限制 | BAM 没有已辨识的飞特 D20 映射；0.8255kg 是拆 EVA 前整机称量；COM/惯量仍有不确定性 |

`params/1910_m6.json` 是运行仿真必须的电机物理配置，`config.json` 是训练超参数，均不是神经网络权重。嘴 ID34 不属于 14 维 actor。训练终止/重新采样属于训练流程，不能拿训练中带 reset 的统计代替不重置验收。

来源与修改边界见 `SOURCE_LINEAGE.json`。BAM 为 Apache-2.0，许可证在 `licenses/`；其余上游来源沿用仓库根 `THIRD_PARTY.md`。本轮验证了随机scratch短训练、47500的精确加载/ONNX对齐和1次继续PPO；没有在另一台电脑或实机验收，没有启动长训练。47500已有基础原生CPU离线资格，但实物站立仍未通过，不把新训练产物或此迁移检查冒称实机合格。
