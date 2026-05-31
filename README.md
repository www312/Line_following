# Follow_lineV2 — 摄像头触发雷达避障巡线小车

STM32F103C8T6 + OpenMV + F302 毫米波雷达 + 八路灰度传感器

## 硬件连接

| 外设 | 接口 | 引脚 | 说明 |
|------|------|------|------|
| 八路灰度 | USART2 | PA2 TX → 模块 RX, PA3 RX ← 模块 TX | 115200 8N1 |
| F302 雷达 | USART1 | PA9 TX, PA10 RX | 115200 8N1, 0xA206 主动上报 |
| OLED | I2C1 | PB6 SCL, PB7 SDA | 128×64 SSD1306 |
| 电机驱动 | Soft I2C | MOTOR_SCL, MOTOR_SDA | |
| **OpenMV 摄像头** | GPIO | **P0 → PB12** | HIGH=T字路口, LOW=清除 |
| **LED 指示灯** | GPIO | **PA6** | RADAR_MODE 闪烁 3s |
| 按钮 (保留) | GPIO | PB13 | 输入上拉 (当前未使用) |

## 状态机总览

```mermaid
stateDiagram-v2
    [*] --> NORMAL : 上电

    state NORMAL {
        [*] --> norm_active
        norm_active : LineWalking() 巡线
        norm_active : UI=传感器画面
        norm_active : suppressed=转弯后2500ms屏蔽PB12
    }

    state RADAR_MODE {
        [*] --> radar_active
        radar_active : LineWalking() 巡线
        radar_active : UI=传感器画面(不变)
        radar_active : PA6 LED闪烁3s
        radar_active : 等all_black(5s超时)
    }

    state STOP_1S {
        [*] --> stop_active
        stop_active : 小车完全停止
        stop_active : UI=雷达画面
        stop_active : 持续1s读雷达
    }

    state AVOID {
        [*] --> avoid_active
        avoid_active : LineWalking() 按避障方向
        avoid_active : UI=雷达画面
        avoid_active : 15s超时恢复原方向
    }

    NORMAL --> RADAR_MODE : PB12↑ && !suppressed && 上电>2s
    RADAR_MODE --> NORMAL : PB12↓ 或 5s超时无all_black
    RADAR_MODE --> STOP_1S : all_black(8路全黑)
    STOP_1S --> AVOID : 1s内有≥1帧有效雷达目标
    STOP_1S --> NORMAL : 1s内无有效目标(保持原方向)
    AVOID --> NORMAL : 15s超时(恢复pre_mode)
```

## LineWalking 行为分支

```mermaid
stateDiagram-v2
    [*] --> PRE

    state PRE {
        pre_calc : 计算 left/right/all_white
        pre_arm  : 路口装弹 & 持锁延长
        pre_supp : 左右转都写last_turn_tick
    }

    PRE --> SYM : left+right≥4 && left==right<br/>对称路口
    PRE --> CENTERED : x4黑&&x5黑&&≤4黑&&!hold
    PRE --> LOST : all_white 完全脱线
    PRE --> PROP : 其余 按比例修正

    state SYM {
        FWD : 向前冲100ms
        CROSS : 判定十字→直行
        T_TURN : 判定T字→vz_search转向
        [*] --> FWD
        FWD --> CROSS : x4&&x5黑
        FWD --> T_TURN : 否则
    }

    CENTERED : vz=0 直行
    LOST : 原地pivot搜索(vz_search)
    PROP : vz=(right-left)×550<br/>限幅±2000<br/>hold钳制±2000
```

## 雷达数据通路

```mermaid
flowchart TD
    subgraph ISR[USART1 中断]
        RX[接收1字节] --> RB[256字节环形缓冲]
        ERR[错误中断] --> ABORT[Abort+重启]
    end

    subgraph PARSE[字节级帧解析 f302_feed_byte]
        H0[WAIT_HDR0] -->|0xAA/0x55| H1[WAIT_HDR1]
        H1 -->|互补头| COLLECT[COLLECT 12字节]
        H1 -->|单头| H1
        COLLECT -->|idx==14| VALID[校验]
        VALID -->|cmd≠0xA206| H0
        VALID -->|Σ错| H0
        VALID -->|通过| EMIT[输出range/angle/have_target]
        EMIT --> H0
    end

    subgraph CAM[OpenMV 摄像头]
        EI[Edge Impulse 48x48 GRAYSCALE] --> CLS{分类}
        CLS -->|curve≥4帧确认| P0_H[P0=HIGH]
        CLS -->|curve消失| P0_L[P0=LOW]
    end

    subgraph TRIGGER[触发链路 main.c]
        PB12[PB12 上升沿] -->|!suppressed&&上电>2s| RADAR_MODE
        RADAR_MODE -->|all_black| STOP[STOP_1S 停车1s]
        STOP -->|有目标| AVOID[AVOID 15s]
        STOP -->|无目标| NORM
    end

    RB -->|F302_Radar_Poll drain| PARSE
    EMIT -->|s_target| STOP
    P0_H --> PB12
```

## 核心常量

### 巡线参数 (line_follow.h)

| 常量 | 值 | 说明 |
|------|----|------|
| `IRR_SPEED` | 150 | 巡线前进速度 |
| `VZ_PER_ERR` | 550 | 比例修正系数 |
| `VZ_SEARCH_LEFT` | -2500 | 脱线左转搜索 |
| `VZ_SEARCH_RIGHT` | +2500 | 脱线右转搜索 |
| `VZ_TURN_LEFT` | -2000 | Hold 钳制左转 |
| `VZ_TURN_RIGHT` | +2000 | Hold 钳制右转 |
| `TURN_HOLD` | 30 帧 | 转弯持锁帧数 |

### 状态机参数 (main.c)

| 常量 | 值 | 说明 |
|------|----|------|
| `SUPPRESS_MS` | 2500 | 转弯后屏蔽 PB12 窗口 |
| `RADAR_MODE_TIMEOUT_MS` | 5000 | RADAR_MODE 等 all_black 超时 |
| `STOP_DURATION_MS` | 1000 | 停车读雷达时间 |
| `AVOID_TIMEOUT_MS` | 15000 | 避障最大持续时间 |
| `STARTUP_IGNORE_MS` | 2000 | 上电忽略 PB12 窗口 |
| `RADAR_DIST_THRESH_CM` | 60 | 有效障碍距离上限 |
| `RADAR_ANGLE_LIMIT` | 6000 | 有效角度 ±60° |

## 文件结构

```
follow_lineV2/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── stm32f1xx_it.h
│   └── Src/
│       ├── main.c              ← 顶层状态机
│       ├── stm32f1xx_hal_msp.c
│       └── stm32f1xx_it.c
├── Lib/
│   ├── Inc/
│   │   ├── line_follow.h       ← 巡线常量 & 接口
│   │   ├── ir_line8.h          ← 八路灰度驱动
│   │   ├── f302_radar.h        ← 雷达协议解析
│   │   ├── f302_radar_uart.h   ← 雷达 UART 驱动
│   │   └── motion_car.h        ← 运动控制
│   └── Src/
│       ├── line_follow.c       ← LineWalking() 行为分支
│       ├── ir_line8.c
│       ├── f302_radar.c        ← 字节级帧解析状态机
│       ├── f302_radar_uart.c   ← 环形缓冲 & ISR
│       └── motion_car.c
├── docs/
│   ├── 01_top_level.puml       ← 顶层 PlantUML
│   ├── 02_line_walking.puml    ← 巡线 PlantUML
│   └── 03_radar_pipeline.puml  ← 雷达 PlantUML
├── CMakeLists.txt
└── README.md                   ← 本文件
```

## OpenMV 端

```
ei-road-openmv-v2-impulse/
├── main.py              ← P0 GPIO 输出
├── trained.tflite       ← Edge Impulse 48×48 灰度模型
└── labels.txt           ← background / curve
```

模型：`background` / `curve` (curve = T 字路口)，连续 4 帧 score≥0.58 确认，P0 拉高。消失后 1800ms cooldown 再允许下次触发。

## 编译

```bash
cmake --preset Debug -S .
cmake --build build/Debug
```
