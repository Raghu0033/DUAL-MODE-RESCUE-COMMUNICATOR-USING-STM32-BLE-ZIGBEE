/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : GPS -> BLUETOOTH -> ZIGBEE (send once per BT burst)
  *                  + Buzzer repeating: 20s ON / 30s OFF using TIM3 + TIM4 PWM
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* (none) */

/* Private define ------------------------------------------------------------*/
#define ZIGBEE_COOLDOWN_MS 7000U   /* 7 seconds cooldown between ZigBee sends per burst */
#define BUZZER_ON_SEC      20U
#define BUZZER_OFF_SEC     30U

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1; /* GPS (USART1) */
UART_HandleTypeDef huart2; /* Bluetooth (USART2) */
UART_HandleTypeDef huart6; /* ZigBee (USART6) */

TIM_HandleTypeDef htim3; /* 1s tick */
TIM_HandleTypeDef htim4; /* PWM for buzzer -> PB6 TIM4_CH1 */

/* UART RX single-byte buffers */
uint8_t uart2_rx_byte = 0; /* Bluetooth incoming byte */
uint8_t gps_rx_byte = 0;   /* GPS incoming byte */

/* GPS parsing buffers/state */
char gps_sentence[128];
int gps_index = 0;
char latest_gps_line[128];
volatile uint8_t gps_line_ready = 0;
volatile double last_lat = 0.0;
volatile double last_lon = 0.0;
volatile uint8_t last_gps_valid = 0;

/* Bluetooth / ZigBee state */
volatile uint8_t msg_received_flag = 0;      /* set when BT receives a printable byte */
volatile uint32_t msg_timestamp_ms = 0;      /* LED timer */
volatile uint8_t zigbee_sent_flag = 0;       /* send only once per BT burst */
volatile uint32_t last_bt_activity_ms = 0;   /* last BT activity time */

/* Buzzer state */
volatile uint32_t buzzer_sec_counter = 0;
volatile uint8_t buzzer_state = 0; /* 0=OFF, 1=ON */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);

/* Utility prototypes */
static double nmea_to_decimal(const char *s);
static int parse_nmea_latlon(const char *line, double *out_lat, double *out_lon);

/* ---------------------------------------------------------------------------*/
/* ---------------------------- Implementation -------------------------------*/
/* ---------------------------------------------------------------------------*/

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init(); /* GPS */
    MX_USART2_UART_Init(); /* Bluetooth */
    MX_USART6_UART_Init(); /* ZigBee */
    MX_TIM3_Init();        /* 1s tick */
    MX_TIM4_Init();        /* PWM buzzer on PB6 */

    /* LED off initially (PC13 active-low on many boards) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    /* Start UART IRQ receptions */
    if (HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1) != HAL_OK) {
        Error_Handler();
    }
    /* ZigBee RX not required for your requested flow (we only TX to zigbee). If you want RX, start here:
       HAL_UART_Receive_IT(&huart6, &zigbee_rx_byte, 1);
    */

    /* Start TIM3 base with interrupt (1 second tick) */
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
        Error_Handler();
    }

    /* Ensure TIM4 PWM is stopped initially (buzzer off) */
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

    /* Main loop: handle flags set by IRQ/callbacks */
    while (1)
    {
        /* If bluetooth activity detected, set LED and attempt zigbee send once per burst */
        if (msg_received_flag) {
            msg_received_flag = 0;
            /* Turn LED ON (active-low) */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            msg_timestamp_ms = HAL_GetTick();
            last_bt_activity_ms = HAL_GetTick();

            if (!zigbee_sent_flag && last_gps_valid) {
                /* Prepare lat/lon message and send to ZigBee (USART6) */
                char outbuf[80];
                int n = snprintf(outbuf, sizeof(outbuf), "LAT: %.6f\r\nLON: %.6f\r\n", last_lat, last_lon);
                if (n > 0) {
                    HAL_UART_Transmit(&huart6, (uint8_t*)outbuf, (uint16_t)n, 300);
                    zigbee_sent_flag = 1;
                }
            }
        }

        /* Clear LED after 2 seconds of inactivity */
        if (msg_timestamp_ms && (HAL_GetTick() - msg_timestamp_ms >= 2000U)) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            msg_timestamp_ms = 0;
        }

        /* Reset zigbee_sent_flag when BT activity has stopped for cooldown period */
        if (zigbee_sent_flag && (HAL_GetTick() - last_bt_activity_ms >= ZIGBEE_COOLDOWN_MS)) {
            zigbee_sent_flag = 0;
            /* optional: require new GPS fix before sending again */
            last_gps_valid = 0;
        }

        /* If a full GPS NMEA line is available, parse and store coordinates */
        if (gps_line_ready) {
            gps_line_ready = 0;
            double lat=0.0, lon=0.0;
            if (parse_nmea_latlon(latest_gps_line, &lat, &lon)) {
                last_lat = lat;
                last_lon = lon;
                last_gps_valid = 1;
                /* We do NOT send GPS to Bluetooth — only to ZigBee when BT triggers. */
            } else {
                /* parse failed - ignore or keep last valid coordinates */
            }
        }

        HAL_Delay(5); /* small sleep */
    }
}

/* -------------------- SystemClock_Config -------------------- */
/* Simple HSI-based clock: SYSCLK = HSI (16 MHz), APB1 = /1, APB2 = /1 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  /* HCLK = 16 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;   /* PCLK1 = 16 MHz -> TIMs = 16 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 16 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/* -------------------- TIM3 init (1 s tick) -------------------- */
static void MX_TIM3_Init(void)
{
    /* TIM3 clock = 16 MHz (APB1 timer clock).
       We want 1 s interrupt:
         prescaler = 16000-1 => 1 kHz tick
         period    = 1000-1  => overflow every 1000 ticks = 1 s
    */
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16000U - 1U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000U - 1U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    /* NVIC for TIM3 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* -------------------- TIM4 PWM (buzzer) init -------------------- */
static void MX_TIM4_Init(void)
{
    /* TIM4 PWM on CH1 (PB6).
       We choose roughly 2 kHz PWM:
         prescaler = 1600-1 => timer tick = 10 kHz
         period = 5-1 => PWM freq = 10k / 5 = 2000 Hz
       Pulse set to 50% (ARR+1)/2
    */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 1600U - 1U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 5U - 1U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    /* Configure PB6 as AF (TIM4_CH1) - AF2 on STM32F4 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim4.Init.Period + 1U) / 2U; /* 50% */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /* Note: do not start PWM here; we will start/stop it in the TIM3 callback */
    HAL_TIM_MspPostInit(&htim4); /* if Cube generated this function, it configures GPIO; safe to call */
}

/* -------------------- UART inits -------------------- */
static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600; /* GPS NEO-6M default */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600; /* HM-10 typical */
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART6_UART_Init(void)
{
    __HAL_RCC_USART6_CLK_ENABLE();
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 9600; /* ZigBee module - change if required */
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}

/* -------------------- GPIO init -------------------- */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* LED PC13 (active-low) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Note:
       - UART pins and TIM4_CH1 pin (PB6) AF config should be generated in HAL MSP init
         (CubeMX does this in stm32f4xx_hal_msp.c). If you manually manage GPIO, ensure:
           PA9/PA10 -> USART1 (or pins you selected)
           PA2/PA3  -> USART2 (or pins you selected)
           PA11/PA12-> USART6 (as your screenshot shows)
           PB6      -> TIM4_CH1 (AF2)
    */
}

/* -------------------- HAL callbacks -------------------- */

/* Called by HAL when a timer with interrupt (TIM3) elapses */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        /* 1 second tick */
        buzzer_sec_counter++;

        if (buzzer_state == 0) {
            /* currently OFF - wait BUZZER_OFF_SEC to start buzzer */
            if (buzzer_sec_counter >= BUZZER_OFF_SEC) {
                buzzer_sec_counter = 0;
                buzzer_state = 1;
                /* start PWM tone */
                HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
            }
        } else {
            /* currently ON - wait BUZZER_ON_SEC to stop buzzer */
            if (buzzer_sec_counter >= BUZZER_ON_SEC) {
                buzzer_sec_counter = 0;
                buzzer_state = 0;
                /* stop PWM tone */
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            }
        }
    }
}

/* UART RX complete callback: we handle GPS bytes and Bluetooth bytes here */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* GPS input */
        if (gps_rx_byte == '\r') {
            /* ignore */
        } else if (gps_rx_byte == '\n') {
            gps_sentence[gps_index] = '\0';
            strncpy(latest_gps_line, gps_sentence, sizeof(latest_gps_line)-1);
            latest_gps_line[sizeof(latest_gps_line)-1] = '\0';
            gps_index = 0;
            gps_line_ready = 1;
        } else {
            if (gps_index < (int)sizeof(gps_sentence)-1) {
                if ((uint8_t)gps_rx_byte >= 0x20) gps_sentence[gps_index++] = gps_rx_byte;
            } else {
                /* overflow protection */
                gps_index = 0;
            }
        }
        /* restart GPS receive */
        HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
        return;
    }

    if (huart->Instance == USART2) {
        /* Bluetooth input: any printable byte => mark activity */
        if ((uint8_t)uart2_rx_byte >= 0x20) {
            msg_received_flag = 1;
        }
        /* restart BT receive */
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
        return;
    }

    /* If using USART6 RX, handle similarly and restart; not needed in current flow */
}

/* Handle UART errors: restart receives */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
    } else if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    } else if (huart->Instance == USART6) {
        /* if you enable USART6 RX, restart it here */
    }
}

/* -------------------- NMEA parsing helpers -------------------- */

/* convert NMEA lat/lon string "ddmm.mmmm" or "dddmm.mmmm" to decimal degrees */
static double nmea_to_decimal(const char *s)
{
    if (!s || *s == '\0') return 0.0;
    double v = atof(s);
    size_t len = strlen(s);
    if (len <= 7) { /* lat: ddmm.mmmm */
        int dd = (int)(v / 100.0);
        double mm = v - dd * 100.0;
        return dd + mm / 60.0;
    } else { /* lon: dddmm.mmmm */
        int ddd = (int)(v / 100.0);
        double mm = v - ddd * 100.0;
        return ddd + mm / 60.0;
    }
}

/* Parse $GPRMC or $GPGGA line, extract lat/lon; returns 1 on success */
static int parse_nmea_latlon(const char *line, double *out_lat, double *out_lon)
{
    if (!line || !out_lat || !out_lon) return 0;

    char buf[128];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char *saveptr = NULL;
    char *tok = strtok_r(buf, ",", &saveptr);
    if (!tok) return 0;

    if (strcmp(tok, "$GPRMC") == 0) {
        /* $GPRMC,time,status,lat,N,lon,E,... */
        (void)strtok_r(NULL, ",", &saveptr); /* time */
        char *status = strtok_r(NULL, ",", &saveptr);
        char *lat_s = strtok_r(NULL, ",", &saveptr);
        char *ns = strtok_r(NULL, ",", &saveptr);
        char *lon_s = strtok_r(NULL, ",", &saveptr);
        char *ew = strtok_r(NULL, ",", &saveptr);
        if (!status || status[0] != 'A') return 0; /* invalid */
        if (!lat_s || !lon_s) return 0;
        double lat = nmea_to_decimal(lat_s);
        double lon = nmea_to_decimal(lon_s);
        if (ns && ns[0] == 'S') lat = -lat;
        if (ew && ew[0] == 'W') lon = -lon;
        *out_lat = lat; *out_lon = lon;
        return 1;
    } else if (strcmp(tok, "$GPGGA") == 0) {
        /* $GPGGA,time,lat,N,lon,E,fix,... */
        (void)strtok_r(NULL, ",", &saveptr); /* time */
        char *lat_s = strtok_r(NULL, ",", &saveptr);
        char *ns = strtok_r(NULL, ",", &saveptr);
        char *lon_s = strtok_r(NULL, ",", &saveptr);
        char *ew = strtok_r(NULL, ",", &saveptr);
        char *fix = strtok_r(NULL, ",", &saveptr);
        if (!fix || fix[0] == '0') return 0;
        if (!lat_s || !lon_s) return 0;
        double lat = nmea_to_decimal(lat_s);
        double lon = nmea_to_decimal(lon_s);
        if (ns && ns[0] == 'S') lat = -lat;
        if (ew && ew[0] == 'W') lon = -lon;
        *out_lat = lat; *out_lon = lon;
        return 1;
    }

    return 0;
}

/* -------------------- Error / assert handlers -------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Blink LED or stay here - debug breakpoint helpful */
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

/* End of file */
