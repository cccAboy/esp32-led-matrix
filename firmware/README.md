## 1. 目录分层

- `components/BSP/`：硬件驱动与板级封装
	- `KEY/`：按键
	- `RGB/`：灯板驱动
	- `mpu6050/`：MPU6050 传感器底层
- `components/Middlewares/`：中间件
	- `FLIP/`：FLIP算法核心
	- `GRAVITY/`：重力方向模块 + MPU6050 重力封装
	- `SIM/`：效果任务（water_sim）——把 gravity + FLIP 输出映射到 RGB 灯板
- `main/`：应用入口与模块编排（`app_main`）

------

## 2. 总体数据流（运行时）

`sim_task()` 以固定帧率循环（`SIM_FPS=45`）：

1. 读重力方向：`gravity_get()` 得到 `gx/gy`（无效则用 0）
2. 推进流体仿真一步：`flip_step(f, dt, gx, gy)`
3. 从仿真取出 16×16 亮度网格：`flip_get_led_grid(f, grid)`
4. 按键切换调色板（GPIO0，低有效）：`key_get_press(&s_key)`
5. 亮度 -> 颜色 LUT -> 映射到灯珠索引 -> `rgb_set()`
6. `rgb_show()` 刷新整屏

------

## 3. 模块说明与对外 API

## 3.1 FLIP（`components/Middlewares/FLIP/flip.c`）

### 目标

实现一个 2D 粒子流体（FLIP/PIC 混合）仿真，并输出一个“可显示的亮度网格”（float）。

### 对外函数

#### 1) 创建 / 销毁

```
FlipFluid* flip_create(float sim_w, float sim_h, int visible_res, float fill_ratio);
void flip_destroy(FlipFluid* f);
```

- `visible_res`：可视分辨率
- 内部仿真分辨率 = `visible_res + 2`（四周 padding 1 格，用作边界）
- `fill_ratio`：初始水位高度比例

#### 2) 设置重力强度

```
void flip_set_gravity_scale(FlipFluid* f, float gravity_scale);
```

water_sim 里设置为 `9.81`（地球）。换成 1.62 就是“月球水”。

#### 3) 推进一步

```
void flip_step(FlipFluid* f, float dt, float gx, float gy);
```

`gx/gy` 是“方向分量”，在内部会乘以 `gravity_scale` 变成真实加速度。

内部大致步骤：

- 积分粒子（加速度 -> 速度 -> 位置）
- 粒子互斥（push apart，防止重叠）
- 边界碰撞（把粒子限制在容器内）
- 粒子->网格速度传输（to_grid，FLIP ratio=0.9）
- 计算粒子密度场（用于显示 + 漂移补偿）
- 压力投影（solve incompressibility，20 iters，over_relaxation=1.9）
- 网格->粒子速度回传（to_particles）

#### 4) 取显示网格

```
void flip_get_led_grid(const FlipFluid* f, float* out_grid);
```

- 输出大小：`(f_num_x-2) * (f_num_y-2)`，也就是可视的 `W*H`
- 输出值范围：大致被映射到 `0..LED_VAL_MAX_F`（
- 内部会做：
	- `particle_density / particle_rest_density`
	- `DENSITY_CLAMP_F=1.2` 做亮度压缩
	- gamma LUT（`GAMMA_F=0.6`）校正
	- 映射到 `0..20`

> 注意：FLIP 的 `get_led_grid()` 写入索引是 `out_grid[i * visible_y + j]`，water_sim 读取也用 `grid[x * H + y]`，两者一致（按 x 主序）。

------

## 3.2 水模拟任务（`components/Middlewares/SIM/water_sim.c`）

### 目标

把 **重力输入** 驱动 **FLIP 仿真**，并把仿真“亮度网格”映射成 RGB 灯板输出；支持按键切换调色板。

### 关键配置

- 网格/灯板：`W=16, H=16, LED_COUNT=256`

- 帧率：`SIM_FPS`（`vTaskDelayUntil` 定帧）

- 亮度等级：`LED_VAL_MAX_I=20` → `LED_LEVELS=21`（0~20）

- 按键：`GPIO_NUM_0`，低有效，消抖 50ms

- LED 索引映射：蛇形走线（偶数行正向，奇数行反向）

	```
	static inline int led_index_row_major(int x, int y) {
	    if (y & 1) return y * W + (W - 1 - x);
	    else       return y * W + x;
	}
	```

### 调色板与 LUT

- `PALETTES[PALETTE_COUNT][PAL_N]`：每个调色板 6 个关键色
- 启动时 `build_palette_lut(i)` 预计算每个亮度等级对应的 RGB（避免每帧插值）
- 按键按下：`s_palette_idx = (s_palette_idx + 1) % PALETTE_COUNT`

### 输出流程（每个像素）

1. 从 FLIP 网格取亮度 `v`
2. clamp 到 `0..LED_VAL_MAX_F`
3. 转成等级 `lv = (int)(v + 0.5f)` 并 clamp 到 `0..LED_LEVELS-1`
4. 取 `s_pal_lut[palette][lv]`
5. `rgb_set(index, r,g,b)` + `rgb_show()`

------

## 3.3 应用入口（`main/app_main`）

当前写死启动水模拟：

```
void app_main(void) {
    gravity_init();
    mpu6050_gravity_start();

    water_sim_start(/*core_id=*/1, /*stack=*/8192, /*prio=*/5);
}
```

后续如果要支持“按键/UDP/NVS 切模式”，建议把 `g_sim_mode` 改为可变，并提供停止/切换机制（比如用事件组或任务通知关停旧 sim 任务）。

------

## 4. 关键参数速查（调效果最常用

### 4.1 流体“手感”

- `flip_set_gravity_scale(f, 9.81f)`：重力强度（越大越“重/粘”冲击越明显）
- `flip_ratio=0.9f`（在 `transfer_velocities` 调用处）：越接近 1 越“保留细节”，但可能更抖；越接近 0 越 PIC 更稳更糊
- `solve_incompressibility num_iters=20`：越大越不“可压缩”，更像水但更耗 CPU
- `push_particles_apart num_iters=2`：太低会粘连，太高会更“硬”

### 4.2 显示“亮度/对比”

在 FLIP 里：

- `DENSITY_CLAMP_F=1.2`：越小越容易变亮（更“满屏”）
- `GAMMA_F=0.6`：gamma 校正，<1 会提升暗部细节
- `LED_VAL_MAX_F=20.0`：最终亮度上限（对应 LUT 的等级）

在 water_sim 里：

- `LED_VAL_MAX_I=20`：亮度等级数（21 档）
- 调色板颜色（`PALETTES`）决定最终风格

------

## 5. 常见注意点

1. **grid 索引顺序**
	 FLIP 输出按 `out_grid[x * H + y]`，water_sim 也按 `grid[x * H + y]` 读取——不要改成 `y*W+x`，否则会“旋转/错位”。
2. **LED 物理走线映射**
	 你现在的 `led_index_row_major()` 是“蛇形行扫描”。如果你的灯板走线不同，优先改这个函数，不要动仿真网格。
3. **性能**
	 `SIM_FPS=60` + 压力迭代 20 次，对 ESP32 仍可能偏紧（取决于优化和其他任务）。如果出现掉帧：

- 先把 `SIM_FPS` 降到 30
- 或把 `solve_incompressibility num_iters` 降到 10~15

