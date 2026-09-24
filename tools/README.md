# PC 诊断与升级工具

`can_test.py` 使用 CANable 的 SLCAN 接口与 ECU 通信，可执行 UDS 功能测试、完整 BIN 固件升级和应用启动验证。

## 安装

需要 Python 3.10 或更高版本。在项目根目录打开 PowerShell：

```powershell
python -m pip install -r .\tools\requirements.txt
```

运行工具前先停止 CANgaroo，避免它占用 CANable 的 COM 口。

## 运行

默认使用 `COM17`、500 kbit/s，并读取应用工程的 Debug BIN：

```powershell
python .\tools\can_test.py
```

也可以显式指定参数：

```powershell
python .\tools\can_test.py `
  --port COM17 `
  --bitrate 500000 `
  --firmware .\F407_Body_ECU\build\Debug\F407_Body_ECU.bin
```

常用菜单：

- `u`：执行完整固件升级。
- `v`：等待 `0x123` 心跳，验证 Bootloader 已跳转到应用程序。
- `f`：显示 BIN 大小、下载地址和 CRC-32。
- `1`～`9`：分步骤测试 UDS 服务和 4 字节下载流程。

完整升级过程见 [固件升级流程](../docs/firmware-update-flow.md)。
