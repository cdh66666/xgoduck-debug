# Host反馈修复候选（未安装、未默认启用）

这三份Controller独立保存，只供审查和离线验证。默认 `robot-app/python/controller.py` 仍为 `181d85760c6fb9be9ed89d96b6f77bc4a2017a183a6ebb3b530189dbc1e5ee78`；本目录不含自动安装器，不会被 `RELEASE_RUNTIME.json` 加载。

| 文件夹 | 作用 | 父版本 |
|---|---|---|
| `candidate/` | zero-only：保留磁盘校准权威、核对后续MCU RAW，确认之前不允许上力 | 默认181d |
| `clock-candidate/` | clock-only：滤波dt使用MCU sample_us，Host monotonic仍负责新鲜度和心跳 | 默认181d |
| `combined-candidate/` | 两项组合 | zero-only上叠加同一clock补丁 |

候选SHA、父SHA和未安装状态见 `MANIFEST.json`；`patches/` 是相对路径增量diff。文件原字节复制，没有调整模型、PD、滤波alpha、物理参数或正常目标生成。

## 已复现的问题与适用前提

zero-only修复的缺陷：首次旧RAW能覆盖内存中的磁盘zero，并把notify立即标成成功。候选把RAM同步请求与后续观测匹配分开；只有后续15轴RAW匹配已保存权威zero，才放行既有ARM。RAW协议没有请求nonce，因此这是后续值核对，不是显式RPC ACK。仅在已卸力且没有未知厂家事务时按最多4Hz重试RAM参考，不重复厂家EEPROM校准。

clock-only修复的是采样时间与送达时间混用。重复/倒退源时间不刷新新鲜度；uint32回绕展开；250ms源缺口仍触发原过滤器重置。相同均匀源输入与基线数值一致，相同源输入批量送达不再改变过滤结果。Host实际到达时间、150ms新鲜度、网页心跳仍保留原依据，没有插帧。

两项都是代码缺陷的离线复现，**尚未证明是本次实物失稳的原因，也没有硬件安装或闭环稳定性验收**。时钟方案保留旧“序号倒退视为MCU重启”的分类，DQS1不能独立区分全部旧epoch重放与真重启；源时间跨半个uint32周期也不能唯一展开。审查/安装前必须核对默认父文件SHA、当前协议和校准台账，不可直接混入不同基线。

## 离线测试（无设备）

```powershell
python -m pip install -r candidates/host-feedback-20261020/requirements-test.txt
python -B candidates/host-feedback-20261020/test_zero_sync.py
python -B candidates/host-feedback-20261020/test_zero_sync.py --candidate combined-candidate
python -B candidates/host-feedback-20261020/test_clock_sync.py
```

脚本执行原样Controller方法AST，用假时钟/Bridge/模型；时钟测试使用仓库当前真实SignalFilter。原始零位表只是合成fixture，不是设备校准数据。所有临时zero文件和测试JSON写入系统临时目录，退出清理，不写机器人或公开源码树。

路径便携适配仅为：定位 `pc/active_runtime.py` 和默认181d、把合成输出改临时目录。测试计算与断言不改。zero-only 12组、combined零位12组、两份时钟各6组回归通过；这些是软件检查，不是实物PASS。
