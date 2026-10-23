# 直接连接配置：当前这只小鸭子的设备实例

设备主人已明确要求将调试所需连接配置直接随仓库提供，不加密、不需要解密口令。它是这只实体鸭子的实例配置，不是所有小鸭子通用的出厂参数。

新电脑克隆仓库后，在根目录运行：

    pwsh -File .\Start-Duck.ps1

脚本安装依赖，把profile里的6个文件直接复制到本机调试台位置，并启动服务；不发送上力、动作、校准或烧录请求。已有完整配置会继续使用；明确更换配置时可运行 `python pc/tools/import_connection.py --replace`。

## 其他人使用自己的设备时修改什么

| 文件 | 换成自己的内容 |
|---|---|
| profile/device.json | 自己主控的serial |
| profile/ssh-identity.json | 同一serial、自己的host_key_alias；保留相对文件名id_ed25519/known_hosts |
| profile/id_ed25519 | 已获自己主控授权的SSH私钥，不能只改IP就沿用别人的设备身份 |
| profile/known_hosts | 自己主控核实过的SSH主机密钥，与host_key_alias对应 |
| profile/wireless-auth.json | 自己板端程序的认证key |
| profile/network.json | 自己主控的局域网地址提示；程序仍会重新核对SSH主机与serial |

复制目的地由 `pc/tools/import_connection.py` 固定映射到device.local.json、tools/.private和qa连接缓存。不要取消主机密钥和设备身份核对。局域网可达性不等于凭据访问控制；此公开实例配置是按设备主人要求提供。板端data、舵机零位、PD和旧动作目标不在这6份连接配置中，不会被导入覆盖。
