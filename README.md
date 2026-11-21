# 电控组第三次培训作业
- **贾政**
- ***20263001720***
## 程序要求
全工程只有一份运行状态`g_run`，按键、串口、OLED、串口反馈都读写这一份数据，因此显示与实际状态不会出现不一致。
```c
typedef struct
{
    uint8_t running;
    uint8_t dir;
    uint16_t step_ms;
    uint8_t led;
} RunState;

static RunState g_run = {1, 0, STEP_DEFAULT, 1};
```
中断回调只完成数据存储和将串口状态改回空闲
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Rb_Put(g_uart_ch);
        HAL_UART_Receive_IT(&huart1, &g_uart_ch, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_tx_busy = 0;
    }
}
```
### 主循环
包含状态更新、数据包校验与解析、发送串口反馈和OLED刷新
```c
while (1)
    {
        if (Key_Edge(&g_key_dir, KEY_DIR_GPIO_Port, KEY_DIR_Pin))
        {
            g_run.dir ^= 1;
            Uart_RequestFeedback();
        }

        if (Key_Edge(&g_key_pause, KEY_PAUSE_GPIO_Port, KEY_PAUSE_Pin))
        {
            g_run.running ^= 1;
            g_led_tick = HAL_GetTick();
            Uart_RequestFeedback();
        }

        if (g_run.running)
        {
            if (HAL_GetTick() - g_led_tick >= g_run.step_ms)
            {
                g_led_tick = HAL_GetTick();
                g_run.led = Led_Next(g_run.led, g_run.dir);
                Led_Show(g_run.led);
            }
        }
        else
        {
            g_led_tick = HAL_GetTick();
        }

        Proto_Poll();
        Feedback_Poll();
        Ui_Refresh();
    }

```
### 串口反馈格式
```
STATE=RUN,DIR=FWD,STEP=300,LED=1
```  
| 字段 | 取值 | 说明 |
|---|---|---|
| `STATE` | `RUN` / `PAUSE` | 运行 / 暂停 |
| `DIR` | `FWD` / `REV` | 正向 / 反向 |
| `STEP` | 100~2000 | 切换间隔（ms） |
| `LED` | 1 / 2 / 3 | 当前点亮的灯号 |
## 1 按键控制流水灯功能
| 命名 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| LED1 / LED2 / LED3 | PA0 / PA3 / PA7 | 推挽输出 | 高电平点亮 |
| KEY_DIR | PB1 | 上拉输入 | 按下为低电平，切换正向与反向 |
| KEY_PAUSE | PB11 | 上拉输入 | 按下为低电平，切换暂停与继续 |

### 流水灯实现
定义流水灯状态，定义流水灯数和间隔时间，初始化结构体
```c
typedef struct
{
    uint8_t running;
    uint8_t dir;
    uint16_t step_ms;
    uint8_t led;
} RunState;

#define LED_N        3
#define STEP_DEFAULT 500

static RunState g_run = {1, 0, STEP_DEFAULT, 1};
```
定义切换流水灯的时刻
```c
static uint32_t g_led_tick = 0;
```
定义函数控制LED点亮与熄灭
```c
static void Led_Show(uint8_t idx)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (idx == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (idx == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (idx == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
```
定义函数算出下一个点亮的LED
```c
static uint8_t Led_Next(uint8_t cur, uint8_t dir)
{
    if (dir == 0)
        return (cur >= LED_N) ? 1 : (uint8_t)(cur + 1);
    return (cur <= 1) ? LED_N : (uint8_t)(cur - 1);
}
```
初始点亮LED1，初始化`g_led_tick`
```c
g_led_tick = HAL_GetTick();
Led_Show(g_run.led);
```
通过`HAL_GetTick()`实现非阻塞延时
```c
if (g_run.running)
        {
            if (HAL_GetTick() - g_led_tick >= g_run.step_ms)
            {
                g_led_tick = HAL_GetTick();
                g_run.led = Led_Next(g_run.led, g_run.dir);
                Led_Show(g_run.led);
            }
        }
        else
        {
            g_led_tick = HAL_GetTick();
        }
```
### 按键控制及按键消抖
定义按键状态,初始化结构体
```c
typedef struct
{
    uint8_t raw;
    uint8_t stable;
    uint32_t t_change;
} KeyState;

static KeyState g_key_dir = {1, 1, 0};
static KeyState g_key_pause = {1, 1, 0};
```
设置两个初始相等的状态变量（raw和stable）和一个时间变量（t_change）  
按按键瞬间，立刻改变第一个状态变量，同时把时间变量改为当前时间  
当两个状态变量不相等且 （当前时间 - 时间变量） 超过了 20ms，同步两个状态变量，认为是一次有效按键
```c
static uint8_t Key_Edge(KeyState* k, GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0 : 1;

    if (raw != k->raw)
    {
        k->raw = raw;
        k->t_change = HAL_GetTick();
        return 0;
    }

    if (raw != k->stable && (HAL_GetTick() - k->t_change) >= 20)
    {
        k->stable = raw;
        return (uint8_t)(k->stable == 0);
    }

    return 0;
}
```
初始化`t_change`
```c
g_key_dir.t_change = HAL_GetTick();
g_key_pause.t_change = HAL_GetTick();
```
通过`Key_Edge`返回的结果控制流水灯,并发送串口反馈
```c
 if (Key_Edge(&g_key_dir, KEY_DIR_GPIO_Port, KEY_DIR_Pin))
        {
            g_run.dir ^= 1;
            Uart_RequestFeedback();
        }

        if (Key_Edge(&g_key_pause, KEY_PAUSE_GPIO_Port, KEY_PAUSE_Pin))
        {
            g_run.running ^= 1;
            g_led_tick = HAL_GetTick();
            Uart_RequestFeedback();
        }
```
## 2 通过串口发送简易数据包控制流水灯
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| USART1 RX | PA10 | 复用浮空输入 | 115200 |
### 串口接收
#### 环形缓冲区
定义缓冲区上限
```c
#define RB_SIZE 32
```
定义指针和数组
```c
static volatile uint8_t g_rb[RB_SIZE];
static volatile uint8_t g_rb_head = 0;
static volatile uint8_t g_rb_tail = 0;
```
定义写入函数`Rb_Put`，计算下一个写入的位置，将接收的数据写入缓冲区
```c
static void Rb_Put(uint8_t b)
{
    uint8_t next = (uint8_t)((g_rb_head + 1) & (RB_SIZE - 1));
    if (next == g_rb_tail) return;
    g_rb[g_rb_head] = b;
    g_rb_head = next;
}
```
定义读取函数`Rb_Get`，将数据从缓冲区取出，计算下一个读取的位置
```c
static uint8_t Rb_Get(uint8_t* b)
{
    if (g_rb_tail == g_rb_head) return 0;
    *b = g_rb[g_rb_tail];
    g_rb_tail = (uint8_t)((g_rb_tail + 1) & (RB_SIZE - 1));
    return 1;
}
```
#### 数据接收
开启串口，将接收的数据放入`g_uart_ch`，每次放一个字节
```c
static uint8_t g_uart_ch = 0;

HAL_UART_Receive_IT(&huart1, &g_uart_ch, 1);
```
通过中断回调，将`g_uart_ch`中的数据放入环形缓冲区
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        Rb_Put(g_uart_ch);
        HAL_UART_Receive_IT(&huart1, &g_uart_ch, 1);
    }
}
```
#### 数据包解析与校验
定义数据包格式和帧头，初始化结构体
```c
#define PKT_LEN  6
typedef struct
{
    uint8_t buf[PKT_LEN];
    uint8_t cnt;
} PktState;

#define PKT_HEAD 0xAA

static PktState g_pkt = {{0}, 0};
```
定义校验函数`Pkt_Checksum`累加数据包1-5项（含帧头），保留低8位
```c
static uint8_t Pkt_Checksum(const uint8_t* p)
{
    uint16_t sum = 0;
    uint8_t i;
    for (i = 0; i < 5; i++) sum += p[i];
    return (uint8_t)(sum & 0xFF);
}
```
定义解析函数`Proto_Poll`，通过读取函数`Rb_Get`取出缓冲区中所有数据，找到帧头打包数据包，数据包满了就重新计数，通过校验函数`Pkt_Checksum`校验，并将返回的结果与校验值比较，校验通过后将数据包交给执行函数`Proto_Apply`
```c
static void Proto_Poll(void)
{
    uint8_t b;

    while (Rb_Get(&b))
    {
        if (g_pkt.cnt == 0)
        {
            if (b == PKT_HEAD) g_pkt.buf[g_pkt.cnt++] = b;
            continue;
        }
        g_pkt.buf[g_pkt.cnt++] = b;

        if (g_pkt.cnt == PKT_LEN)
        {
            g_pkt.cnt = 0;
            if (g_pkt.buf[5] == Pkt_Checksum(g_pkt.buf))
                Proto_Apply(g_pkt.buf);
        }
    }
}
```
#### 执行
定义执行函数`Proto_Apply`，检查数据包合法性，通过后按数据包内容更新流水灯，并发送串口反馈
```c
static void Proto_Apply(const uint8_t* p)
{
    uint8_t run = p[1];
    uint8_t dir = p[2];
    uint16_t ms = (uint16_t)((uint16_t)p[3] | ((uint16_t)p[4] << 8));

    if (run > 1 || dir > 1) return;
    if (ms < 100 || ms > 2000) return;


    g_run.running = run;
    g_run.dir = dir;
    g_run.step_ms = ms;
    g_led_tick = HAL_GetTick();

    Uart_RequestFeedback();
}
```
## 3 通过OLED显示当前状态
### 必做
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| OLED SCL / SDA | PB6 / PB7 | 开漏输出 | 软件 I²C |  

[OLED.c](./Core/Src/OLED.c)  
[OLED.h](./Core/Inc/OLED.h)  
[OLED_Font.h](./Core/Inc/OLED_Font.h)  
文件来源：26电控组招新-群文件-第三次培训所用资料.zip    
更改了引脚配置
#### 实现原理
定义流水灯改变前的状态，初始化状态
```c
typedef struct
{
    uint8_t running;
    uint8_t dir;
    uint16_t step_ms;
    uint8_t led;
} RunState;

static RunState g_ui_last;
static uint8_t  g_ui_valid = 0;
```
定义刷新函数`Ui_Refresh`，设置一个缓冲区`buf`，如果流水灯状态没有改变则直接结束函数，如果改变则先将流水灯状态翻译成字符串放入缓冲区，再根据字符串刷新OLED屏幕
```c

static void Ui_Refresh(void)
{
    char buf[8];
    uint8_t need_all;

    if (g_ui_valid &&
        g_ui_last.running == g_run.running &&
        g_ui_last.dir     == g_run.dir     &&
        g_ui_last.step_ms == g_run.step_ms &&
        g_ui_last.led     == g_run.led)
    {
        return;
    }

    need_all = (uint8_t)(!g_ui_valid ||
                         g_ui_last.running != g_run.running ||
                         g_ui_last.dir     != g_run.dir     ||
                         g_ui_last.step_ms != g_run.step_ms);

    g_ui_last  = g_run;
    g_ui_valid = 1;

    if (need_all)
    {
        OLED_ShowString(1, 1, "STATE:");
        OLED_ShowString(1, 7, g_run.running ? "RUN  " : "PAUSE");

        OLED_ShowString(2, 1, "DIR:  ");
        OLED_ShowString(2, 7, g_run.dir ? "REV" : "FWD");

        snprintf(buf, sizeof buf, "%4ums", (unsigned)g_run.step_ms);
        OLED_ShowString(3, 1, "STEP: ");
        OLED_ShowString(3, 7, buf);
    }

    snprintf(buf, sizeof buf, "%u", (unsigned)g_run.led);
    OLED_ShowString(4, 1, "LED:  ");
    OLED_ShowString(4, 7, buf);
}
```
### 选做
之后补上
## 4 通过串口中断发送反馈
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| USART1 TX | PA9 | 复用推挽 | 115200 |
### 串口反馈格式
```
STATE=RUN,DIR=FWD,STEP=300,LED=1
```  
| 字段 | 取值 | 说明 |
|---|---|---|
| `STATE` | `RUN` / `PAUSE` | 运行 / 暂停 |
| `DIR` | `FWD` / `REV` | 正向 / 反向 |
| `STEP` | 100~2000 | 切换间隔（ms） |
| `LED` | 1 / 2 / 3 | 当前点亮的灯号 |
### 必做
定义缓冲区，初始化串口状态
```c
static char g_tx_buf[64];
static volatile uint8_t g_tx_busy = 0;
static volatile uint8_t g_tx_pending = 0;
```
定义请求函数`Uart_RequestFeedback`，提示串口发送反馈
```c
static void Uart_RequestFeedback(void)
{
    g_tx_pending = 1;
}
```
定义发送函数`Feedback_Poll`，有反馈需要发送且串口空闲时，通过`snprintf`将流水灯当前状态打包成文字，再通过`HAL_UART_Transmit_IT`发送反馈
```c
static void Feedback_Poll(void)
{
    int n;

    if (!g_tx_pending || g_tx_busy) return;
    g_tx_pending = 0;

    n = snprintf(g_tx_buf, sizeof g_tx_buf,
                 "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                 g_run.running ? "RUN" : "PAUSE",
                 g_run.dir ? "REV" : "FWD",
                 (unsigned)g_run.step_ms,
                 (unsigned)g_run.led);

    if (n <= 0 || n >= (int)sizeof g_tx_buf) return;

    g_tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart1, (const uint8_t*)g_tx_buf, (uint16_t)n) != HAL_OK)
        g_tx_busy = 0;
}
```
通过中断回调，在发送完成后，将串口状态改为空闲
```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1)
    {
        g_tx_busy = 0;
    }
}
```
### 选做
之后补上

