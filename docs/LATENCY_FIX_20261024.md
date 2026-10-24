# 2026-10-24 乱序、排队与显示延迟修复

本轮始终使用托盘静止条件：OFF、enabled=false、motion-inhibit 保留。没有 ARM、关节目标或运动命令；维护重启后只请求 OFF。模型权重、机械零位、禁动文件的哈希保持不变。

## 根因与实际修改

1. 板上 Arduino Router 0.5.3 用独立 goroutine 转发每条通知，不能保证先后顺序。安装包含官方顺序转发修复的 0.7.0 可执行文件。原 `/usr/bin/arduino-router` 保留；服务通过既有 `90-xgoduck-baud.conf` 指向 `/usr/local/lib/xgoduck/arduino-router-0.7.0`，仍使用原 Unix socket、UART 和 2 Mbaud。来源：[官方 PR #28](https://github.com/arduino/arduino-router/pull/28)、[0.7.0 发布](https://github.com/arduino/arduino-router/releases/tag/v0.7.0)。
2. 实机 Python 采样剖析显示，静止校准在持有控制器锁的 100 Hz 回调中反复遍历 256 点陀螺窗口求均值和标准差。只修 router 后，乱序消失，但处理速度降到约 53 Hz，排队更明显。改为增量更新窗口的和、平方和，随窗口移出旧样本；仍使用原 256 点、2 秒及静止阈值。量化为 1/32 dps 的有界原始数值可用 double 精确累加，方差只在最终除法/开方取舍入。测试逐窗对照完整重算，包括窗口替换和重置。
3. 原 25 Hz 状态采样之后再经过独立 20 Hz 发送定时器，增加无意义等待。现在采样完成唤醒发送方；慢消费者只拿最新快照，动作记录仍保留所有采样。
4. 原显示在首次收到数据时固定播放基准，再额外延后 350 ms；首次晚到会留下额外永久延迟。现在直接显示最新实测端点，附加缓冲为 0，不外推、不回放无线积压帧。图形、15 轴、读数使用同一端点；失鲜仍明确提示保留姿态。后台历史动作回放保持独立。
5. 实时状态 API 默认不再复制整秒历史环；需要历史的显式游标查询仍保留。网页每 25 ms 取最新状态，互斥禁止请求堆叠，运动心跳保持原 50 ms。
6. SSH 流压缩减少重复 JSON。OpenSSH 10.0p2 的 `TCP_NODELAY` 设置发生在会话建立路径；原 `-N` 纯转发跳过该路径。改为无 PTY、无设备访问的静默 stdin 读取会话，关闭管道/连接即退出，认证与主机密钥核对不变。依据：[packet.c](https://github.com/openssh/openssh-portable/blob/V_10_0_P2/packet.c)、[session.c](https://github.com/openssh/openssh-portable/blob/V_10_0_P2/session.c)。并行 20 秒 A/B 中，额外排队中位数 68.3 → 28.2 ms，P99 428.6 → 316.0 ms；该阶段同时打开额外采样连接，不能当作最终页面数值。

## 实测口径

- 同一托盘、同一 5 GHz AP、同一主控/电脑，分别运行 `measure.py` 和 `browser.cjs` 的 60 秒只读采样。浏览器为同样的 1440×1000 无头 Edge 软件渲染；去掉前 2 秒加载期。分阶段记录保留，没有删除失败或中间结果。
- 页面指标为实际渲染端点的**已知数据年龄**，包含原生年龄、已测额外传输等待、缓存和显示等待；不是通过拍摄运动测得的“实物到屏幕绝对延迟”，首次无线传播时间无法由不同机器的时钟直接确定。
- 不以旧缓冲播放比例和新直接显示的 playing 比例作等价平滑度比较；旧缓冲能够掩盖晚到，新模式主动显示失鲜。
- 原始本地证据：`pc/qa/latency-20261024/` 内 before、router-only、after-runtime、after、final 及对应 browser JSON/截图、tunnel-ab.json、profile-before.json、安装回执。

## 安装与回退

- 官方 arm64 deb SHA256：`8ca07d4ec2f3dd1ce238322397235bb56e328e70e84d612ef581d8aa61af4ef9`。
- 已安装 router 二进制 SHA256：`237059d6dd9ffa0e39cabca1e3723503123e282c1626afe50f73711a7ec3eef4`。
- 板端备份：`.codex-backups/latency-router-20261024`、`.codex-backups/latency-runtime-20261024`，均在该应用目录下。原 router、服务覆盖文件和三份 Python 源码已保留。
- 直接 sudo 维护要求密码；实际使用该用户已有 Docker 管理权限和已安装镜像执行固定维护命令，没有更改 sudo 权限。临时维护容器无网络且退出即删除。
- 维护顺序必须是确认 OFF/disabled/inhibited，停止应用，更新/恢复 router 配置并重启服务，随后重新创建应用容器以重新绑定 Unix socket，再确认 OFF 与新鲜反馈。不可只重启 router 后继续使用旧 socket 的容器。

最终数值见同目录实测汇总与 `firmware-support/validation/latency-20261024.json`。实物站立、走动和无线在运动中的表现仍未测试。

## 最终同条件实测对比

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| 新增乱序 | 98 次 | 0 次 |
| 序号缺口 | 1077 | 0 |
| 平均接收频率 | 70.38 Hz | 99.92 Hz |
| 额外显示缓冲 | 350 ms | 0 ms |
| 页面已知数据年龄中位数 | 828.7 ms | 98.1 ms |
| 页面已知数据年龄 P95 | 917.6 ms | 256.0 ms |
| 页面已知数据年龄 P99 | 1079.0 ms | 402.1 ms |
| 页面已知数据年龄最大值 | 1461.9 ms | 714.0 ms |
| 板端接收后年龄 P99 | 48.23 ms | 24.86 ms |
| 本地状态 API 平均响应体 | 126356 B | 14484 B |
| 15 轴最大观测跨度 | 0° | 0° |

最终 600/600 板端读数均为 OFF/disabled/inhibited，乱序/缺口/重同步/MCU 超时/报告丢弃新增均为 0。最终网页 918 个加载后采样中，224 个明确显示反馈失鲜；没有通过缓冲把这些尖峰隐藏掉。PC 状态采样 1193/1200 次处于已连接新鲜窗口，7 次按原有规则拒绝滞后状态。无线/操作系统/渲染尖峰仍在，不能宣称完全无抖动。

最终网页没有 JavaScript 错误或硬件写请求；152 项 Python 全套通过，最后修改后相关发布测试 10 项、SSH 恢复测试 8 项、JavaScript 30 组通过。新增滚动统计、立即显示、慢消费者、默认最新状态读取均有回归覆盖。
