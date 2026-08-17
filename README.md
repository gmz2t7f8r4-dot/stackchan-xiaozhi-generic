# StackChan Xiaozhi Generic

[中文说明](README_zh.md) · [日本語](README_ja.md)

面向 M5Stack CoreS3 StackChan 的通用增强固件，基于
[xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 与
[Stackchan-HtSz](https://github.com/mo-hantang/Stackchan-HtSz) 开发。

公开版本不包含任何私人角色名称、人格提示、照片、设备标识、Wi-Fi
凭据、服务器地址、访问令牌或私有表情素材。使用者可以在自己的小智服务端
和资源包中自由设置角色。

## 主要功能

- CoreS3 屏幕手势、SI12T 摸头、BMI270 体感交互
- SCS 总线舵机动作、空闲动画与本机轻量人脸跟随
- WS2812 情绪灯效与 MCP 灯光控制
- 相机实时取景、手动/自动拍照与视觉理解接口
- 照片以标准 MCP `image` 内容返回（客户端支持时可直接显示原图）
- 本机连续舞步 MCP 工具
- M5Stack 原厂 StackChan World App 的 BLE 舞蹈协议兼容
- BLE 连接后可切换回表情主页而不断开手机连接
- 声音模式、闹钟、工作日问候及中国时区/SNTP 修正
- 针对 CoreS3 内部 SRAM、I²C 并发与舵机/灯带峰值功耗的稳定性保护

## 隐私与安全

仓库只包含源代码，不包含已配网设备的 NVS、编译产物或服务器部署密钥。

- 不要提交 `.env`、私钥、API Key、Wi-Fi 密码或设备白名单。
- 摄像头与 MCP 服务不要无鉴权暴露在公网。
- 建议为视觉接口和 MCP 接口启用 Bearer Token，并在反向代理层限制来源。
- 刷写前请检查 `sdkconfig.defaults` 中的 OTA 地址，将示例地址替换为自己的服务。

## 编译

建议使用 ESP-IDF v5.5.x：

```bash
git clone <your-repository-url>
cd stackchan-xiaozhi-generic

idf.py set-target esp32s3
idf.py build
```

低内存电脑可以限制并行任务：

```bash
idf.py build -- -j1
```

默认目标为 M5Stack CoreS3。首次构建时，ESP-IDF Component Manager 会下载
公开依赖；这些依赖遵循各自的许可证。

## 配置

通用版默认唤醒词为“你好小智”。可以在 `sdkconfig.defaults` 中修改：

```ini
CONFIG_CUSTOM_WAKE_WORD="ni hao xiao zhi"
CONFIG_CUSTOM_WAKE_WORD_DISPLAY="你好小智"
```

视觉分析地址应由小智服务端下发，或者在自己的配置流程中调用
`Camera::SetExplainUrl` 设置。公开源码不会内置私人服务器。

## 刷写

完整首次刷写：

```bash
idf.py -p <PORT> flash monitor
```

如果设备已有兼容分区表，开发阶段也可以只写应用分区。请先核对自己的
分区表，不能把示例偏移量盲目用于其他硬件或固件版本。

## 原厂 App 蓝牙舞蹈

1. 在 StackChan 上进入“原厂 App 自设舞蹈”页面。
2. 在 StackChan World App 的 Dance 页面选择 `Bluetooth`。
3. 连接 `StackChan`，选择音乐并录制动作。
4. 连接成功后可将本机滑回表情主页；BLE 会话保持，表情和舵机数据继续生效。

进入舞蹈页不会关闭网络会话、停止唤醒词或重建语音模型；歌曲播放与页面导航
互不干扰。为了降低瞬时功耗，固件会限制原厂 App 舵机、表情和灯带数据的刷新率。

## MCP 能力

主要新增工具包括：

- `self_camera_take_photo(question)`
- `self_motion_dance(sequence, tempo_ms, repeat)`
- `self_motion_stop_dance()`
- 屏幕表情、舵机方向、灯光与声音模式相关工具

具体工具清单由固件在运行时发布，以实际构建配置为准。

## 上游与许可

本项目不是从零编写的独立固件，而是多个开源项目上的衍生开发。原文件中的
版权声明和许可证必须保留。主体与新增代码按 MIT License 发布；当前 CoreS3
构建会静态链接 GPL-3.0 的 SCServo_lib 衍生驱动，因此分发该完整固件时应按
GPL-3.0 提供对应源码。第三方组件、字体、表情和工具遵循各自许可证。详见
[NOTICE.md](NOTICE.md)。

本项目与 M5Stack 官方无隶属或背书关系；`M5Stack`、`StackChan` 及相关商标
归其权利人所有。

## 硬件安全

不要在舵机通电受控时强行扭动机身。首次测试自定义动作时请使用低速、小角度，
并确保设备放在稳定平面上。
