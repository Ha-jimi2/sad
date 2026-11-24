# 电控组第三次培训作业
- **贾政**
- ***20263001720***
> `LED_KING/` 子目录为**必做版**（软件 I²C 驱动 OLED + 串口中断反馈）  
> `LED_QUEEN/` 子目录为**选做版**（硬件 I²C 驱动 OLED + DMA 发送反馈）
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
## 串口反馈格式
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

[OLED.c](LED_KING/Core/Src/OLED.c)  
[OLED.h](LED_KING/Core/Inc/OLED.h)  
[OLED_Font.h](LED_KING/Core/Inc/OLED_Font.h)  
文件来源：26电控组招新-群文件-第三次培训所用资料.zip    
更改了引脚配置
#### main.c
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
#### OLED.c
定义两个宏，便于改变SCL和SDA的电平
```c
#define OLED_W_SCL(x)		HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, (GPIO_PinState)(x))
#define OLED_W_SDA(x)		HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, (GPIO_PinState)(x))
```
START条件：SCL高电平时SDA由高变低
```c
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}
```
STOP条件：SCL高电平时SDA由低变高
```c
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

```
定义发送函数`OLED_I2C_SendByte`，在SCL低电平时改变SDA的电平以发送字节，然后拉高SCL让OLED读取字节，发送结束后释放SDA、拉高SCL，OLED主动拉低SDA表示收到（ACK），检查ACK的代码没写
```c
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
    OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SCL(0);
}
```
定义2个函数，分别用于写入命令和写入数据，控制字节`0x00`表示后面是命令，控制字节`0x40`表示后面是数据
```c
/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}
```
定义定位函数`OLED_SetCursor`，确定光标位置，将坐标转化为命令，发给OLED  
Y 是页地址（0~7），每页 8 行像素；X 是列地址（0~127）。列地址是 7 位而命令只能带 4 位参数，所以拆成高 4 位、低 4 位两次发送
```c
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}
```
定义绘制函数`OLED_ShowChar`，算出绘制的位置，调用定位函数`OLED_SetCursor`发给OLED，再把绘制的字符拆成数据，调用`OLED_WriteData`发给OLED  
8×16 字体占 16 行，而显存一页只有 8 行，所以一个字符跨 2 页，需要两次定位；Char - ' ' 把 ASCII 码转成字库下标
```c
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
	}
}
```
定义函数`OLED_ShowString`用于把单词拆成字符，再调用绘制函数`OLED_ShowChar`
```c
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}
```
### 选做
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| OLED SCL | PB6 | 复用开漏（I2C1_SCL） | 硬件 I²C，Fast Mode 400 kHz |
| OLED SDA | PB7 | 复用开漏（I2C1_SDA） | 同上 |
#### main.c
同必做
#### OLED.c
定义OLED地址
```c
#define OLED_ADDR  0x78
```
定义发送函数`OLED_HW_Write`，定义一个缓冲区`tmp`，将控制字节`ctrl`和要发送的字节先后填入缓冲区，然后调用`HAL_I2C_Master_Transmit`发送给OLED
```c
static void OLED_HW_Write(uint8_t ctrl, const uint8_t* p, uint16_t len)
{
	static uint8_t tmp[1 + 128];
	if (len > 128) return;
	tmp[0] = ctrl;
	for (uint16_t i = 0; i < len; i++) tmp[1 + i] = p[i];
	HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, tmp, (uint16_t)(len + 1), 100);
}
```
发送命令改为调用发送函数`OLED_HW_Write`发送
```c
void OLED_WriteCommand(uint8_t Command) { OLED_HW_Write(0x00, &Command, 1); }
```
数据改为由发送函数`OLED_HW_Write`批量发送
## 4 通过串口中断发送反馈
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| USART1 TX | PA9 | 复用推挽 | 115200 |
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
| 功能 | 引脚 | 模式 | 备注 |
|---|---|---|---|
| USART1 TX | PA9 | 复用推挽 | 115200，由DMA搬运 |  

| 功能 | 外设 / 通道 | 方向 | 模式 / 备注 |
|---|---|---|---|
| 反馈发送 | USART1_TX → **DMA1 Channel4** | Memory → Peripheral | Normal，Byte，Priority Low |
| 中断 | `USART1 global interrupt` | Enabled | 提示发送完成 |
| 中断 | `DMA1 Channel4 interrupt` | Enabled | DMA 传输完成 |
更改发送函数`Feedback_Poll`，改为通过`HAL_UART_Transmit_DMA`发送反馈
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
    if (HAL_UART_Transmit_DMA(&huart1, (const uint8_t*)g_tx_buf, (uint16_t)n) != HAL_OK)
        g_tx_busy = 0;
}
```

