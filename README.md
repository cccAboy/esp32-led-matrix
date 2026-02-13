# ESP32 LED Matrix（16×16 WS2812）

这是一个以 **算法** 为核心的 LED 矩阵效果项目：使用 ESP32 驱动 16×16 WS2812 灯板，提供一套可扩展的实时效果框架。  
当前示例包含“流体/水波”类效果（参考 Flip 思路实现），后续会持续加入更多不同风格的效果与玩法。

- 演示视频：【用esp32和flip把灯板变成会流动的水】 https://www.bilibili.com/video/BV1BoFoz6EGF/?share_source=copy_web&vd_source=7e1d93ad9ac103c6c0653b43edfdaebd
- 代码仓库：<https://github.com/cccAboy/esp32-led-matrix>
- 嘉立创链接：https://oshwhub.com/ccbaw123/deng-ban-flip



## 作者的话：

这个仓库是我在 ESP32 + 16×16 RGB 灯板上做的一套小型“效果引擎”：目前主打 **重力感应水波（MPU6050）**，整体结构尽量写得 **新手也能看懂、能改得动**，后续也会继续加入更多效果（比如火焰、爱心等），把它做成一个“拿来就能玩”的小项目。

如果这个项目对你有帮助——哪怕只是让你少踩一个坑——**麻烦右上角点个 Star ⭐** 支持一下！
 Star 就是我继续更新和维护的最大动力，也欢迎：

- 提 Issue：遇到问题/想要的功能都可以说
- PR：一起把效果做得更酷更顺滑
- 分享：把你的成品图/视频贴出来，我超爱看！

再次感谢大家，Let’s make it glow ✨



## 快速上手

1. 按照docs文件夹里面的购置清单购买器件

2. 在嘉立创打板（每月能免费打板两次），打板文件在嘉立创链接里直接打即可，或者在Hardware文件夹里下载

3. 焊接（器件较小，需要加热台和焊接烙铁）并将MPU6050黏贴在灯板背后

4. 软件烧录：将Firmwarel文件夹里的代码拉取下来，编译烧录

	（如不知道如何操作，推荐参考正点原子ESP32快速入门（IDF版），从0操作，到第9讲即可使用）

5. 按照Hardware项目文档里面的接线图将硬件连接起来

6. 上电，恭喜你已经完成项目

> 注意：
>
> **烧录阶段**：先通过 USB 给开发板烧录程序，**烧录完成后请断开 USB（拔掉电脑连接）**。
>
> **上电接线**：**断电状态下完成所有接线**，确认无短路/反接后，再给电源模块上电。
>
**使用阶段**：**设备运行时**不要让开发板连接在电脑 USB 上（避免外部电源与电脑 USB 供电冲突，或异常电流回灌，导致电脑 USB 口受损，或者把开发版烧了）。
> 
作者烧了块开发板，很难闻。







---

## Highlight

- **算法驱动的实时效果**：效果在主控端实时计算并渲染到矩阵像素
- **可扩展效果框架**：方便新增/切换不同效果
- **传感器交互**：可接入 MPU6050 等，让画面随倾斜/晃动产生响应
- **适配 WS2812 矩阵**：支持蛇形走位映射与像素坐标转换

---

## 仓库结构

- `firmware/`：ESP32 固件与算法
- `hardware/`：硬件
- `docs/`：文档资源

---
## License

- Firmware/Code: MIT License (see `firmware/LICENSE`)
- Hardware: CC BY-NC 3.0 (see `hardware/LICENSE`)
- Docs: CC BY-NC 3.0 (see `docs/LICENSE`)

Note: Third-party libraries/components remain under their respective licenses.


---
## 致谢

https://www.youtube.com/watch?v=XmzBREkK8kY



