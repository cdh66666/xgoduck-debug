# 当前固件的依赖源码交接

此目录补齐应用源之外的实际构建依赖，全部是源码、许可证、哈希与离线测试；不含固件二进制、编译缓存、模型、设备身份、密钥、校准数据或上传器。依赖沿用已安装的冻结输入；各次修改与安装记录分开保留。

**2026-10-24 已完成静止安装：** QMI 机身轴取反在 `int16=-32768` 时饱和至 `32767`，避免满量程符号翻转。此前全部65,536值通过桌面 C++ 测试；本次完成实际 ARM 编译、ELF 检查、OpenOCD 上传和新启动核对，安装 ELF 为 `3ebc523e713fd4398807e7a19b8934ba96c30b7c6611a5c8b066ed7dec7ec9ad`。托盘禁动保持，未验证运动效果。历史记录不改写；本轮见 `validation/static-20261024.json` 和 [实际结果](../docs/STATIC_DEBUG_20261024.md)。

## 绑定对象

历史已安装的固件（只记录指纹，不随包提供二进制）：

- ELF SHA256：`0df0fe95af681625ca440435f10cfc9cfed6191ba669895ef2a03af19736bb8f`
- ZSK SHA256：`a1f0e4ab9eeb680c06968c77d784a6ad1d470332901a7274f80bbf9d72c9b1b9`
- 两者历史大小均为121724字节；历史ARM编译exit 0、危险外部短跳转重定位清单为空。
- 官方运行链来源commit：`9dc36787bc61b102c2da0f97c9c758e639e90ab8`。
- FTServo来源commit：`64922cda46e56b21b8c1d9e830d936a1941645ae`，当前 `Read` 已修改，不能称未改厂家SDK。

`BUILD_BINDING.json`保留历史安装pointer、ARM审查及安装receipt的SHA指纹、库版本、历史编译sketch哈希和证据边界；原始带设备信息的receipt没有复制。`SOURCE_LOCK.json`逐文件锁定当前目录输入；全部路径相对本目录。

| 冻结库 | library.properties版本 | 交接方式 |
|---|---|---|
| Arduino_RouterBridge | 0.4.3 | 实际编译的完整源码快照，含有界begin、单次本地bind、只登记RPC接口 |
| Arduino_RPClite | 0.3.1 | 实际编译的完整源码快照，含现有有界事务/帧处理修改 |
| FTServo | 2.0.0 | 实际编译的完整源码及明确的SCS.cpp前后diff |
| MsgPack | 0.4.2 | 实际依赖源码；剔除一个上游自带的Mach-O测试可执行文件 |
| ArxContainer | 0.7.0 | 实际依赖源码 |
| ArxTypeTraits | 0.3.2 | 实际依赖源码 |
| DebugLog | 0.8.4 | 实际依赖源码 |

原审查依赖库存201条，本包保留200条，唯一剔除项是 `MsgPack/examples/tests/cpp/main`，它不是库编译源码；其指纹与理由在BUILD_BINDING。7份库许可证原字节保留。版本字段是上游元数据，不能单独识别本地修改，必须同时使用源码SHA。没有取得可证明的RouterBridge/RPClite干净上游对照，因此提供完整修改后源码，**不伪造“相对原版”补丁或上游commit**。

## 布局和使用

```text
firmware-support/
  sketch/                         # 实际编译的11份应用C++/头文件/构建profile
  firmware_dependencies/libraries/ # 实际冻结的7个库
  baseline/                       # 仅供首包修复原生A/B的两个明确旧源码
  patches/                        # 从上面真实前后源码生成的两个最小diff
  tests/                          # Fake UART、时钟和路由夹具，没有设备传输
  BUILD_BINDING.json
  SOURCE_LOCK.json
```

优先整套使用本目录的 `sketch/` 与相邻 `firmware_dependencies/`。`sketch/sketch.yaml`已经通过相对路径选中这些库，不要只将新sketch配给未修改的全局Arduino库。若迁入另一项目，保持这两个目录的相邻关系，或同步调整profile后另行记录构建输入；不要靠全局覆盖库造成依赖混用。

实际ARM输入profile为 `arduino:zephyr:unoq`、平台 `arduino:zephyr (1.0.0)`。需要事先配置相应Arduino平台和工具链，本包不包含板支持包、Zephyr核心、编译器和Arduino CLI。已有该平台的环境可手动编译：

```powershell
python -B firmware-support/verify_sources.py
arduino-cli compile --profile default --build-path firmware-support/.build-unoq firmware-support/sketch
```

这是编译命令，没有上传或设备操作。另一套安装路径、编译器或板支持包不保证产生逐字节相同ELF；新构建应自行记录完整工具链和新artifact指纹。10月23日实际ARM编译与静止安装记录见 `validation/imu-sync-install-20261023.json`。

重要区别：应用清单中原 `sketch/sketch.yaml` SHA是 `89cec193…`，实际编译profile为相对冻结依赖后的 `58f2d25c…`，完整哈希在BUILD_BINDING。其余实际编译C++/头文件按该文件逐项绑定。此目录单独保留实际构建输入，不自动替换 `robot-app/` 或修改其根清单。

## 补丁内容

- `patches/custom-parser.patch`：`scs_bus.h`连续多FF时保持双FF包头同步，普通合法流不改变。
- `patches/ftservo-read.patch`：FTServo `SCS.cpp::Read`最多再消费10个多余FF，保留ID、长度、checksum、status核对；不修改Ack/Ping和整体事务超时。
- `baseline/sketch/scs_bus.h` 与 `baseline/FTServo/src/SCS.cpp` 是本次修复前的明确实际版本，供检查diff或原生对照；不是声称整套依赖都等于干净上游。无需将旧源码应用回当前固件。
- RouterBridge/RPClite的历史修改采用完整源码交付。它们提供 `beginBounded`、`bindLocalOnce`、`registerOnly` 等当前sketch需要的接口；仅下载相同library.properties版本不能替代。

FTServo遵循随库MIT许可证；RouterBridge/RPClite遵循随库MPL-2.0，其源码头部版权和许可证条款保持。其余库的MIT许可证也完整保留。应用sketch及测试沿用仓库来源说明；本目录不改变第三方许可。

## 原生验证

在已具备C++17编译器的终端中运行；Windows可用Visual Studio Developer PowerShell，或传入 `--compiler`，MSVC需先配置对应开发环境：

```powershell
python -B firmware-support/run_native.py
```

脚本先核对源码与“实际函数逐字提取”绑定，再把六组原生程序编译到系统临时目录、运行并回收，结果存 `validation/native-result.json`。不执行Arduino CLI、串口、SSH、网络或舵机命令。已有便携包原生回归：

- 自定义实际解析函数：3672998断言、1845313次普通流前后一致比较，人工三FF恢复15轴。
- 实际FTServo `SCS.cpp`/`SCSerial.cpp` 原版与修复版：各716079断言；修改版额外FF预算为10。
- 实际RouteStartup头：526断言；RouteRefresh头：13组检查。
- 实际QMI驱动：复现未锁定读取的跨样本字节混合，验证同步锁定、无新数据、短读恢复和有界命令握手。

SDK测试的HLSCL构造/EnableTorque两段从真实HLSCL源码逐字提取，厂家校准头用于Fake UART断言；这是数字夹具，不会对真实电机发送上力或卸力。原生结果中的 `deployed=false` 是该测试自身不部署的声明，历史安装状态另见BUILD_BINDING，不应混读。

本次首次运行时本机编译器诊断文本遇到默认编码解码警告，编译与五组测试均exit 0；runner现显式解码并容忍诊断编码差异。测试算法和源码未因此修改，未重复ARM构建。

这些检查证明源码、解析与启动状态机边界，不证明ARM时序、无线连续性或实物站立稳定。后两项仍未通过，不能因为源码交接完整就标为稳定版。

## 2026-10-23 IMU同步读取修复

按QST QMI8658C Rev B第13节，I²C初始化关闭内部AHB时钟门控，启用六轴同步锁定；读取STATUSINT锁住当前样本，必要时等待12μs，再连续读完12字节并释放锁。命令完成/ACK等待各最多20ms。量程、ODR、轴映射、滤波阈值及机械零位未改。

新旧固件指纹分别保留；旧指纹与 `compiled_sketch_sha256` 是历史记录。新固件经过ARM编译、外部短跳转重定位检查及真实OFF状态验证，不包含运动验收。
