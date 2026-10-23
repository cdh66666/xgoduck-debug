# 已打包的连接配置

`private-connection.enc` 包含这只鸭子的连接配置，使用AES-256-GCM认证加密及PBKDF2-SHA256（600000次、随机salt）派生密钥。解密口令由项目主人在原交接对话中单独提供，不在仓库里。

新电脑在仓库根运行 `pwsh -File .\Start-Duck.ps1`，按提示粘贴口令。脚本会安装调试台依赖、导入设备身份/SSH私钥/固定主机密钥/板端认证/地址提示，然后启动调试台。不再要求从旧电脑另拷ZIP，不会更改机器人或自动上力。

手动导入：`python pc/tools/import_connection.py`。自动化可临时设置环境变量 `XGODUCK_CONNECTION_PASSWORD`；不要将口令写进仓库或命令脚本。导入器只在电脑写配置，不发设备请求。已有不同配置默认拒绝覆盖，明确备份后才用 `--replace`。
