# 位置符号解码候选（未部署）

默认MCU文件与固件不变。此目录仅独立函数和无设备原生测试，没有接入运行链或自动安装器。

现 `hostWord` 按二补码解释bit15；厂家HLSCL::ReadPos使用bit15方向加15bit幅值。例：0x8001厂家为−1，原helper为−32767。候选仅按该厂家语义解码，原样源码SHA见MANIFEST。

测试验证7个厂家黄金值，原函数在4个负方向样本不一致；0..4095全部4096个正常正单圈值完全兼容。本次失败记录反推的位置均在正域约1062..3805，未触发此负方向缺陷，**不能把它当161°误差或站立失稳的根因**。

候选未做板端编译/安装/多圈完整坐标合同验收；不能宣告由一个decode函数就支持完整多圈ABI。没有更改零位、PD、舵机模式、符号配置或默认MCU。

可在已配置C++17编译器环境中离线构建，构建目录放仓库外：

```powershell
cmake -S candidates/servo-position-decoder-20261020 -B ../duck-position-build
cmake --build ../duck-position-build --config Release
ctest --test-dir ../duck-position-build -C Release --output-on-failure
```

只有合成整数样本，无Arduino、串口、网络或EEPROM操作。Windows也可在Developer PowerShell使用 `cl /EHsc /std:c++17 test_position_decode.cpp`，请把输出路径指向临时构建目录。
