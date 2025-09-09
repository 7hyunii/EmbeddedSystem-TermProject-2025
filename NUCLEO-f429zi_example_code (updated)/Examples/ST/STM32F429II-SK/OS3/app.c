#include <includes.h> // uC/OS-III 헤더 파일
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include <stdint.h>
#include <stdio.h>

// --- 태스크 설정 ---
#define APP_CFG_TASK_START_STK_SIZE         128u
#define APP_CFG_TASK_BUTTON_STK_SIZE        128u
#define APP_CFG_TASK_FAN_CTRL_STK_SIZE      128u
#define APP_CFG_TASK_ULTRASONIC_STK_SIZE    128u

#define APP_CFG_TASK_BUTTON_PRIO        5u
#define APP_CFG_TASK_FAN_CTRL_PRIO      6u
#define APP_CFG_TASK_ULTRASONIC_PRIO    7u
#define APP_CFG_TASK_START_PRIO         8u

#define BUTTON_TASK_PERIOD_MS           50u  // 버튼 스캔 주기 (ms)
#define FAN_CONTROL_TASK_PERIOD_MS      100u // 팬 제어 주기 (ms)
#define ULTRASONIC_TASK_PERIOD_MS       200u // 0.2초마다 거리 측정

// --- HC-SR04 핀 정의 ---
#define TRIG_PIN                        GPIO_Pin_13
#define ECHO_PIN                        GPIO_Pin_12
#define TRIG_PORT                       GPIOD
#define ECHO_PORT                       GPIOD
#define SENSOR_GPIO_CLK                 RCC_AHB1Periph_GPIOD

// --- 핀 정의 (모터) ---
#define MOTOR_ENA_PIN                   GPIO_Pin_5
#define MOTOR_IN1_PIN                   GPIO_Pin_15
#define MOTOR_IN2_PIN                   GPIO_Pin_13
#define MOTOR_ENA_PORT                  GPIOB
#define MOTOR_IN1_PORT                  GPIOB
#define MOTOR_IN2_PORT                  GPIOB
#define MOTOR_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define MOTOR_ENA_PIN_SOURCE            GPIO_PinSource5
#define MOTOR_ENA_AF_TIM                GPIO_AF_TIM3

// --- 핀 정의 (버튼/LED) ---
#define BTN1_PIN                        GPIO_Pin_10 // On/Off 버튼
#define BTN2_PIN                        GPIO_Pin_12 // 풍속 조절 버튼
#define BTN3_PIN                        GPIO_Pin_14 // 타이머 조절 버튼
#define BTN_PORT                        GPIOE
#define BTN_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED_R_PIN                       GPIO_Pin_15
#define LED_G_PIN                       GPIO_Pin_10
#define LED_B_PIN                       GPIO_Pin_11
#define LED_R_PORT                      GPIOE
#define LED_G_PORT                      GPIOB
#define LED_B_PORT                      GPIOB
#define LED_GPIO_CLK                    (RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOB)


// --- PWM 설정 ---
#define PWM_TIMER                       TIM3
#define PWM_TIMER_CLK                   RCC_APB1Periph_TIM3
#define PWM_PERIOD                      16799

#define SYSTEM_CORE_CLOCK_MHZ           168

typedef struct {
    CPU_BOOLEAN fan_on;      // 선풍기 전원 상태 (DEF_TRUE: On, DEF_FALSE: Off)
    uint8_t     speed_level; // 풍속 (0: 정지, 1: 1단, 2: 2단, 3: 3단)
} Fan_Control_TypeDef;       // 선풍기 제어 상태



static void AppTaskStart(void* p_arg);
static void AppTaskCreate(void);

// 애플리케이션 태스크
static void AppTaskButton(void* p_arg);
static void AppTaskFanControl(void* p_arg);
static void AppTaskUltrasonic(void* p_arg);
static void AppTaskTimerControl (void* p_arg);

// 하드웨어 초기화 함수
static void Setup_Hardware(void);
static void Setup_GPIO(void);
static void Setup_Motor_PWM(void);


// 제어 함수
static void Set_Motor_Speed(uint8_t level);
static void Set_LED_Mode(uint8_t level);
static int  Measure_Distance_mm(void); 
static void delay_us(uint32_t us);


// --- 태스크 TCB 및 스택 ---
static OS_TCB   AppTaskStartTCB;
static CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB   AppTaskButtonTCB;
static CPU_STK  AppTaskButtonStk[APP_CFG_TASK_BUTTON_STK_SIZE];

static OS_TCB   AppTaskFanControlTCB;
static CPU_STK  AppTaskFanControlStk[APP_CFG_TASK_FAN_CTRL_STK_SIZE];

static OS_TCB   AppTaskUltrasonicTCB; 
static CPU_STK  AppTaskUltrasonicStk[APP_CFG_TASK_ULTRASONIC_STK_SIZE]; 

static OS_TCB   AppTaskTimerTCB;
static CPU_STK  AppTaskTimerStk[128u];

volatile Fan_Control_TypeDef g_fan_control;

#define MAX_TIMER_SECONDS 12

volatile uint32_t fan_timer_seconds = 0;  // 남은 시간
volatile CPU_BOOLEAN timer_active = DEF_FALSE;

static OS_SEM AppFanCtrlSem;


int main(void)
{
    OS_ERR err;

    BSP_IntDisAll(); 
    CPU_Init();
    Mem_Init();
    Math_Init(); 

    OSInit(&err); 

    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR )AppTaskStart,
                 (void       *)0u,
                 (OS_PRIO     )APP_CFG_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0u],
                 (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE)APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0u,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);

    (void)&err;
    return (0u);

}

static void AppTaskStart(void* p_arg)
{
    OS_ERR err;
    (void)p_arg;

    BSP_Init();
    BSP_Tick_Init();

    Setup_Hardware();

    g_fan_control.fan_on = DEF_FALSE;
    g_fan_control.speed_level = 0;
    fan_timer_seconds = 0;
    timer_active = DEF_FALSE;

    OSSemCreate((OS_SEM *)&AppFanCtrlSem,
                (CPU_CHAR *)"App Fan Control Sem",
                (OS_SEM_CTR)1u,
                (OS_ERR *)&err);

    AppTaskCreate();

    OSTaskSuspend((OS_TCB *)0u, &err);
}

static void AppTaskCreate(void)
{
    OS_ERR err;

    OSTaskCreate((OS_TCB     *)&AppTaskButtonTCB,
                 (CPU_CHAR   *)"App Task Button",
                 (OS_TASK_PTR )AppTaskButton,
                 (void       *)0u,
                 (OS_PRIO     )APP_CFG_TASK_BUTTON_PRIO,
                 (CPU_STK    *)&AppTaskButtonStk[0u],
                 (CPU_STK_SIZE)APP_CFG_TASK_BUTTON_STK_SIZE / 10u,
                 (CPU_STK_SIZE)APP_CFG_TASK_BUTTON_STK_SIZE,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0u,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskFanControlTCB,
                 (CPU_CHAR   *)"App Task Fan Control",
                 (OS_TASK_PTR )AppTaskFanControl,
                 (void       *)0u,
                 (OS_PRIO     )APP_CFG_TASK_FAN_CTRL_PRIO,
                 (CPU_STK    *)&AppTaskFanControlStk[0u],
                 (CPU_STK_SIZE)APP_CFG_TASK_FAN_CTRL_STK_SIZE / 10u,
                 (CPU_STK_SIZE)APP_CFG_TASK_FAN_CTRL_STK_SIZE,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0u,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskUltrasonicTCB,
                 (CPU_CHAR   *)"App Task Ultrasonic",
                 (OS_TASK_PTR )AppTaskUltrasonic,
                 (void       *)0u,
                 (OS_PRIO     )APP_CFG_TASK_ULTRASONIC_PRIO,
                 (CPU_STK    *)&AppTaskUltrasonicStk[0u],
                 (CPU_STK_SIZE)APP_CFG_TASK_ULTRASONIC_STK_SIZE / 10u,
                 (CPU_STK_SIZE)APP_CFG_TASK_ULTRASONIC_STK_SIZE,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0u,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSTaskCreate((OS_TCB     *)&AppTaskTimerTCB,
                 (CPU_CHAR   *)"App Task Timer",
                 (OS_TASK_PTR )AppTaskTimerControl,
                 (void       *)0u,
                 (OS_PRIO     )9u,
                 (CPU_STK    *)&AppTaskTimerStk[0u],
                 (CPU_STK_SIZE)128u / 10u,
                 (CPU_STK_SIZE)128u,
                 (OS_MSG_QTY  )0u,
                 (OS_TICK     )0u,
                 (void       *)0u,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

}

static void AppTaskButton(void* p_arg)
{
    OS_ERR err;
    uint8_t btn1_prev = 1;
    uint8_t btn2_prev = 1;
    uint8_t btn3_prev = 1;
    (void)p_arg;

    while (DEF_TRUE)
    {
        uint8_t btn1_curr = GPIO_ReadInputDataBit(BTN_PORT, BTN1_PIN);
        uint8_t btn2_curr = GPIO_ReadInputDataBit(BTN_PORT, BTN2_PIN);
        uint8_t btn3_curr = GPIO_ReadInputDataBit(BTN_PORT, BTN3_PIN);

        // 버튼 1 (On/Off)
        if (btn1_prev == 1 && btn1_curr == 0)
        {
            OSSemPend((OS_SEM *)&AppFanCtrlSem,
                      (OS_TICK   )0u,
                      (OS_OPT    )OS_OPT_PEND_BLOCKING,
                      (CPU_TS   *)0u,
                      (OS_ERR   *)&err);
            if (err == OS_ERR_NONE) {
                g_fan_control.fan_on = !g_fan_control.fan_on;

                if (g_fan_control.fan_on) {
                    if (g_fan_control.speed_level == 0) {
                        g_fan_control.speed_level = 1; // 켜지면 기본 1단
                    }
                } else {
                    g_fan_control.speed_level = 0;
                    fan_timer_seconds = 0;
                    timer_active = DEF_FALSE;
                }
                OSSemPost((OS_SEM *)&AppFanCtrlSem,
                          (OS_OPT    )OS_OPT_POST_1, 
                          (OS_ERR   *)&err);
            }
        }

        // 버튼 2 (풍속 조절)
        if (btn2_prev == 1 && btn2_curr == 0){      
            OSSemPend((OS_SEM *)&AppFanCtrlSem,
                      (OS_TICK   )0u,
                      (OS_OPT    )OS_OPT_PEND_BLOCKING,
                      (CPU_TS   *)0u,
                      (OS_ERR   *)&err);
            if (err == OS_ERR_NONE) {
                if (g_fan_control.fan_on) 
                {
                    g_fan_control.speed_level++;
                    if (g_fan_control.speed_level > 3) {
                        g_fan_control.speed_level = 1; // 3단 -> 1단
                    }
                }
                OSSemPost((OS_SEM *)&AppFanCtrlSem,
                          (OS_OPT    )OS_OPT_POST_1,
                          (OS_ERR   *)&err);
            }
        }

        // 버튼 3 (타이머 조절)
        if (btn3_prev == 1 && btn3_curr == 0) {
            OSSemPend((OS_SEM *)&AppFanCtrlSem,
                      (OS_TICK   )0u,
                      (OS_OPT    )OS_OPT_PEND_BLOCKING,
                      (CPU_TS   *)0u,
                      (OS_ERR   *)&err);
            if (err == OS_ERR_NONE) {
                if (fan_timer_seconds + 2 < MAX_TIMER_SECONDS) {
                    fan_timer_seconds += 2;
                    if (!timer_active)
                        timer_active = DEF_TRUE;
                }
                OSSemPost((OS_SEM *)&AppFanCtrlSem,
                          (OS_OPT    )OS_OPT_POST_1,
                          (OS_ERR   *)&err);
            }
        }
        btn3_prev = btn3_curr;


        btn1_prev = btn1_curr;
        btn2_prev = btn2_curr;

        OSTimeDlyHMSM(0u, 0u, 0u, BUTTON_TASK_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

static void AppTaskFanControl(void* p_arg)
{
    OS_ERR err;
    Fan_Control_TypeDef local_fan_state;
    (void)p_arg;

    while(DEF_TRUE)
    {
        OSSemPend((OS_SEM *)&AppFanCtrlSem,
                  (OS_TICK   )0u,
                  (OS_OPT    )OS_OPT_PEND_BLOCKING,
                  (CPU_TS   *)0u,
                  (OS_ERR   *)&err);
        if (err == OS_ERR_NONE) {
            local_fan_state = g_fan_control;
            OSSemPost((OS_SEM *)&AppFanCtrlSem,
                      (OS_OPT    )OS_OPT_POST_1,
                      (OS_ERR   *)&err);
        } else {
            local_fan_state.fan_on = DEF_FALSE;
            local_fan_state.speed_level = 0;
        }

        Set_Motor_Speed(local_fan_state.speed_level);
        Set_LED_Mode(local_fan_state.speed_level);

        OSTimeDlyHMSM(0u, 0u, 0u, FAN_CONTROL_TASK_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

static void AppTaskUltrasonic(void* p_arg)
{
    OS_ERR err;
    CPU_BOOLEAN is_fan_on;
    int distance_mm;
    (void)p_arg;

    while (DEF_TRUE)
    {
        OSSemPend((OS_SEM *)&AppFanCtrlSem,
                  (OS_TICK   )0u,
                  (OS_OPT    )OS_OPT_PEND_BLOCKING,
                  (CPU_TS   *)0u,
                  (OS_ERR   *)&err);
        if (err == OS_ERR_NONE) {
            is_fan_on = g_fan_control.fan_on;
            OSSemPost((OS_SEM *)&AppFanCtrlSem,
                      (OS_OPT    )OS_OPT_POST_1,
                      (OS_ERR   *)&err);
        } else {
            is_fan_on = DEF_FALSE;
        }

        if (is_fan_on)
        {
            distance_mm = Measure_Distance_mm();
            if (distance_mm >= 0 && distance_mm > 300)
            {
                OSSemPend((OS_SEM *)&AppFanCtrlSem,
                          (OS_TICK   )0u,
                          (OS_OPT    )OS_OPT_PEND_BLOCKING,
                          (CPU_TS   *)0u,
                          (OS_ERR   *)&err);
                if (err == OS_ERR_NONE) {
                    g_fan_control.fan_on = DEF_FALSE;
                    g_fan_control.speed_level = 0;
                    fan_timer_seconds = 0;
                    timer_active = DEF_FALSE;
                    OSSemPost((OS_SEM *)&AppFanCtrlSem,
                              (OS_OPT    )OS_OPT_POST_1,
                              (OS_ERR   *)&err);
                }
            }
        }

        OSTimeDlyHMSM(0u, 0u, 0u, ULTRASONIC_TASK_PERIOD_MS, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

static void Setup_Hardware(void)
{
    Setup_GPIO();
    Setup_Motor_PWM();
}
static void Setup_GPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(MOTOR_GPIO_CLK | BTN_GPIO_CLK | LED_GPIO_CLK | SENSOR_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_IN1_PIN | MOTOR_IN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MOTOR_IN1_PORT, &GPIO_InitStructure);

    GPIO_SetBits(MOTOR_IN1_PORT, MOTOR_IN1_PIN);
    GPIO_ResetBits(MOTOR_IN2_PORT, MOTOR_IN2_PIN);

    GPIO_InitStructure.GPIO_Pin = MOTOR_ENA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(MOTOR_ENA_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MOTOR_ENA_PORT, MOTOR_ENA_PIN_SOURCE, MOTOR_ENA_AF_TIM);

	GPIO_InitStructure.GPIO_Pin = BTN1_PIN | BTN2_PIN | BTN3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BTN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_R_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_R_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_G_PIN | LED_B_PIN;
    GPIO_Init(LED_G_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(LED_R_PORT, LED_R_PIN);
    GPIO_ResetBits(LED_G_PORT, LED_G_PIN);
    GPIO_ResetBits(LED_B_PORT, LED_B_PIN);
}

static void Setup_Motor_PWM(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(PWM_TIMER_CLK, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0; // 초기 PWM 0%

    TIM_OC2Init(PWM_TIMER, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);
    TIM_Cmd(PWM_TIMER, ENABLE);
}

static void Set_Motor_Speed(uint8_t level)
{
    uint16_t pulse = 0;
    switch (level) {
        case 1: pulse = PWM_PERIOD * 3 / 5;     break; // 약 60%
        case 2: pulse = PWM_PERIOD * 4 / 5; break;     // 약 80%
        case 3: pulse = PWM_PERIOD;         break;     // 100%
        default: pulse = 0; break;                     // 정지
    }
    TIM_SetCompare2(PWM_TIMER, pulse);
}

static void Set_LED_Mode(uint8_t level)
{
    GPIO_ResetBits(LED_R_PORT, LED_R_PIN);
    GPIO_ResetBits(LED_G_PORT, LED_G_PIN);
    GPIO_ResetBits(LED_B_PORT, LED_B_PIN);

    switch (level) {
        case 1: GPIO_SetBits(LED_G_PORT, LED_G_PIN); break; // 1단: 초록
        case 2: GPIO_SetBits(LED_B_PORT, LED_B_PIN); break; // 2단: 파랑
        case 3: GPIO_SetBits(LED_R_PORT, LED_R_PIN); break; // 3단: 빨강
        default: break;
    }
}


static void delay_us(uint32_t us)
{
    us *= (SYSTEM_CORE_CLOCK_MHZ / 8);
    while (us--)
    {
        __NOP();
    }
}

static int Measure_Distance_mm(void)
{
    uint32_t time = 0;
    uint32_t timeout = 50000;

    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    delay_us(2);
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    delay_us(10);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == Bit_RESET)
    {
        if (--timeout == 0) return -1;
        delay_us(1);
    }

    // Echo 핀이 High인 시간을 측정
    time = 0;
    timeout = 50000;
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == Bit_SET)
    {
        if (--timeout == 0) return -1;
        time++;
        delay_us(1);
    }

    return (int)(time * 343 / 2000);
}

static void AppTaskTimerControl(void* p_arg)
{
    OS_ERR err;
    (void)p_arg;
    CPU_BOOLEAN local_timer_active;
    uint32_t local_fan_timer_seconds;
    uint8_t current_speed;

    while (DEF_TRUE)
    {
        OSSemPend((OS_SEM *)&AppFanCtrlSem,
                  (OS_TICK   )0u,
                  (OS_OPT    )OS_OPT_PEND_BLOCKING,
                  (CPU_TS   *)0u,
                  (OS_ERR   *)&err);
        if (err == OS_ERR_NONE) {
            local_timer_active = timer_active;
            local_fan_timer_seconds = fan_timer_seconds;
            current_speed = g_fan_control.speed_level;
            OSSemPost((OS_SEM *)&AppFanCtrlSem,
                      (OS_OPT    )OS_OPT_POST_1,
                      (OS_ERR   *)&err);
        } else {
            local_timer_active = DEF_FALSE;
            local_fan_timer_seconds = 0;
            current_speed = 0;
        }

        if (local_timer_active)
        {
            if (local_fan_timer_seconds > 0)
            {
                Set_LED_Mode(0);
                OSTimeDlyHMSM(0,0,1,0, OS_OPT_TIME_HMSM_STRICT, &err);

                OSSemPend((OS_SEM *)&AppFanCtrlSem,
                          (OS_TICK   )0u,
                          (OS_OPT    )OS_OPT_PEND_BLOCKING,
                          (CPU_TS   *)0u,
                          (OS_ERR   *)&err);
                if (err == OS_ERR_NONE) {
                    fan_timer_seconds--;
                    OSSemPost((OS_SEM *)&AppFanCtrlSem,
                              (OS_OPT    )OS_OPT_POST_1,
                              (OS_ERR   *)&err);
                }
                Set_LED_Mode(current_speed);
                OSTimeDlyHMSM(0,0,0,500, OS_OPT_TIME_HMSM_STRICT, &err);
            }
            else
            {
                OSSemPend((OS_SEM *)&AppFanCtrlSem,
                          (OS_TICK   )0u,
                          (OS_OPT    )OS_OPT_PEND_BLOCKING,
                          (CPU_TS   *)0u,
                          (OS_ERR   *)&err);
                if (err == OS_ERR_NONE) {
                    g_fan_control.fan_on = DEF_FALSE;
                    g_fan_control.speed_level = 0;
                    timer_active = DEF_FALSE;
                    OSSemPost((OS_SEM *)&AppFanCtrlSem,
                              (OS_OPT    )OS_OPT_POST_1,
                              (OS_ERR   *)&err);
                }
            }
        }
        OSTimeDlyHMSM(0,0,0,100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}
