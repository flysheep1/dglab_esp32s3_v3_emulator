# ESP32-S3 DG-LAB V3 BLE Protocol Emulator

这是一个基于 ESP-IDF + NimBLE 的 ESP32-S3 BLE 协议模拟器。

## 当前功能

- BLE 广播名称：`47L121000`
- GATT Service：
  - `0x180C`
    - `0x150A` WRITE：接收 V3 指令
    - `0x150B` NOTIFY：返回 B1
  - `0x180A`
    - `0x1500` READ / NOTIFY：模拟电量
- 解析 V3：
  - B0（20 字节）
  - BF（7 字节）
- B0 中的 A/B 强度仅在软件变量中模拟，不连接任何刺激/执行硬件。
- 当 B0 修改强度且序列号非 0 时，返回 B1。
- 串口日志会打印收到的数据和解析结果。

## 编译

建议 ESP-IDF 5.x。

```bash
cd dglab_esp32s3_v3_emulator
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

Linux/macOS 例如：

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## 手机测试

推荐使用 nRF Connect。

扫描附近 BLE 设备，找到：

```text
47L121000
```

连接后应该看到：

```text
180C
 ├─ 150A  WRITE
 └─ 150B  NOTIFY

180A
 └─ 1500  READ / NOTIFY
```

在 150B 上打开 Notify。

然后可以向 150A 写入 V3 B0 数据。

例如仓库中的 B0 示例：

```text
B00000000A0A0A0A000A141E0000000000000065
```

注意：该示例只会被本项目作为协议数据解析，不会产生任何实际输出。

## 关于官方 App

这个工程实现的是公开协议层。能否被特定官方 App 完整识别，还取决于 App 是否存在额外的设备识别逻辑。

本项目首先用于验证：

1. ESP32-S3 BLE 广播
2. GATT 服务发现
3. 150A 写入
4. 150B Notify
5. V3 B0/BF 协议

## 安全说明

本工程故意不实现任何物理刺激、升压、驱动或执行器输出。`A/B strength` 只是内存中的模拟数值。
