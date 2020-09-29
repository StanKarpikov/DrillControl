/**
 * @file      main.c
 *
 * @author    Stanislav Karpikov
 *
 * @brief     Common application file
 */

/*---------------------------------------------------------------------------*/
/*---------------------------  INCLUDES  ------------------------------------*/
/*---------------------------------------------------------------------------*/

#include "SparkFun_APDS9960.h"
#include "soft_i2c.h"
#include "stm8s.h"
#include "stm8s_adc1.h"
#include "stm8s_clk.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_it.h"
#include "stm8s_rst.h"
#include "stm8s_tim1.h"

/*---------------------------------------------------------------------------*/
/*---------------------------  DEFINES  -------------------------------------*/
/*---------------------------------------------------------------------------*/

#undef HARDWARE_I2C            /**< Define to use the hardware I2C */
#define LED_WITH_SHIFTREGISTER /**< HC595 shift register is used to control the LEDs */

#define CCR1_Val ((u16)500) /**< Configure channel 1 Pulse Width */
#define CCR2_Val ((u16)250) /**< Configure channel 2 Pulse Width */
#define CCR3_Val ((u16)750) /**< Configure channel 3 Pulse Width */

#define PWM_PRES (120) /**< PWM prescaller */

#define B1_LED (6) /**< LED 1 shift register output position */
#define B2_LED (2) /**< LED 2 shift register output position */
#define B3_LED (4) /**< LED 3 shift register output position */

#define LED_POSITIVE (0x1) /**< Two color LED control: first color */
#define LED_NEGATIVE (0x2) /**< Two color LED control: second color  */

#define TIM1_PERIOD (124) /**< PWM timer period */

/* Input-output ports and pins */
#ifndef LED_WITH_SHIFTREGISTER
#define L1_PORT GPIOC      /**< LED1 port */
#define L1_PIN  GPIO_PIN_5 /**< LED1 pin */
#define L2_PORT GPIOC      /**< LED2 port */
#define L2_PIN  GPIO_PIN_2 /**< LED2 pin */
#define L3_PORT GPIOE      /**< LED3 port */
#define L3_PIN  GPIO_PIN_5 /**< LED3 pin */
#endif

#define B1_PORT GPIOC      /**< Button 1 port */
#define B1_PIN  GPIO_PIN_4 /**< Button 1 pin */
#define B2_PORT GPIOC      /**< Button 2 port */
#define B2_PIN  GPIO_PIN_6 /**< Button 2 pin */
#define B3_PORT GPIOC      /**< Button 3 port */
#define B3_PIN  GPIO_PIN_7 /**< Button 3 pin */

#define STCP_PORT GPIOE      /**< STCP port */
#define STCP_PIN  GPIO_PIN_5 /**< STCP pin */
#define SHCP_PORT GPIOC      /**< SHCP port */
#define SHCP_PIN  GPIO_PIN_1 /**< SHCP pin */
#define D0_PORT   GPIOC      /**< D0 port */
#define D0_PIN    GPIO_PIN_2 /**< D0 pin */

#define FAST_SWITCH_PORT   GPIOD      /**< FAST_SWITCH port */
#define FAST_SWITCH_PIN    GPIO_PIN_7 /**< FAST_SWITCH pin */
#define PROXIMITY_OUT_PORT GPIOD      /**< PROXIMITY_OUT port */
#define PROXIMITY_OUT_PIN  GPIO_PIN_0 /**< PROXIMITY_OUT pin */

#define DELAY595  (250)  /**< Delay used while writing shift register value */
#define DELAY_ADC (1000) /**< Delay used while reading ADC value */

#define DEFAULT_RUN_TIME        (37000) /**< Default run time of the motor */
#define RUN_TIME_MULTIPLICATION (5)     /**< Multiplication of the motor run time (fast mode) */

#define BUTTON_DEBOUNCE_TIME  (200) /**< Debounce time, ms: use to prevent false activation */
#define SENSOR_ACQUIRE_PERIOD (200) /**< Time period for reading data from the proximity sensor, ms */

#define PROXIMITY_THRESHOLD_PWM_CONSTANT (1500.0f) /**< PWM constant to norm the proximity resistor value */

#define IWDG_RELOAD_VALUE (0xaa) /**< Reload value for the IWDG */

/*---------------------------------------------------------------------------*/
/*-------------------------  PRIVATE TYPES  ---------------------------------*/
/*---------------------------------------------------------------------------*/

/** Speed modes */
enum
{
    IDLE,        /**< Idle, wait for a button */
    FIRST_MODE,  /**< Button 1 speed */
    SECOND_MODE, /**< Button 2 speed */
    THIRD_MODE   /**< Button 3 speed */
} speed_modes_t;

/*---------------------------------------------------------------------------*/
/*-------------------------  PRIVATE DATA  ----------------------------------*/
/*---------------------------------------------------------------------------*/

static SparkFun_APDS9960 apds = SparkFun_APDS9960();
static uint8_t proximity_data = 0;
static uint8_t pd = 0;
static uint8_t Buttns[3] = {0, 0, 0};      //Current buttons state
static uint8_t old_Buttns[3] = {0, 0, 0};  //Old buttons state
static uint32_t need = 0;
static uint32_t get = 0;
static uint32_t pwm = 0;

static uint32_t time_to_run = DEFAULT_RUN_TIME;  //Motor ON time

/*---------------------------------------------------------------------------*/
/*----------------------  PRIVATE FUNCTIONS  --------------------------------*/
/*---------------------------------------------------------------------------*/

/**
  * @brief  Configure system clock to run at Maximum clock speed and output the 
  *         system clock on CCO pin
  */
static void CLK_Config(void)
{
    BOOL status = FALSE;

    CLK_DeInit();

    /* NOTE: Use default clock settings. Uncomment for the additional clock control */

    // /* Configure the Fcpu to DIV1*/
    // CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);

    // /* Configure the HSI prescaler to the optimal value */
    // CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);

    // /* Output Fcpu on CLK_CCO pin */
    // CLK_CCOConfig(CLK_OUTPUT_CPU);

    // /* Configure the system clock to use HSE clock source and to run at 24Mhz */
    // status = CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);

    // while (ButtonPressed == FALSE)
    // {
    // }
    // /* Configure the system clock to use HSI clock source and to run at 16Mhz */
    // status = CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
}

/**
  * @brief  Configure TIM4 to generate an update interrupt each 1ms 
  */
static void TIM1_Config(void)
{
    /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

    /* Time base configuration */
    TIM1_TimeBaseInit(128, TIM1_COUNTERMODE_DOWN, TIM1_PERIOD, 0);
    /* Clear TIM4 update flag */
    TIM1_ClearFlag(TIM1_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);

    /* enable interrupts */
    enableInterrupts();

    /* Enable TIM4 */
    TIM1_Cmd(ENABLE);
}

/**
  * @brief  ADC1 configuration
  */
static void ADC1_Config(void)
{
    GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
    GPIO_Init(GPIOE, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);

    ADC1->CR1 = 0x61;                                        // 0b01100001;   // enable ADC
    ADC1->CR2 = 0x08;                                        // 0b00001000;   // right alignment
    ADC1->CR3 = 0x00;                                        // 0;            // data bufer disable
    ADC1->TDRL = (1 << 0) | (1 << 2) | (1 << 4) | (1 << 9);  // 0b11000000;  // disable Schmitt triggers
}

/**
  * @brief  Get value from ADC
  * 
  * @param ChanelNumb Channel number to measure
  * @return ADC value
  */
uint16_t GetADCvalue(uint8_t ChanelNumb)
{
    uint16_t tmphvalue;
    uint16_t tmplvalue;

    ADC1->CSR = ChanelNumb;

    delayX(DELAY_ADC);

    ADC1->CR1 |= 0x61;
    while (!(ADC1->CSR & ADC1_CSR_EOC))
    {
        ;
    }
    ADC1_ClearITPendingBit(ADC1_IT_EOC);

    tmplvalue = ADC1->DRL;
    tmphvalue = (uint16_t)ADC1->DRH << 8;
    tmphvalue = tmphvalue + tmplvalue;

    return (tmphvalue);
}

/**
  * @brief Delay with processor loops
  * 
  * @param loop_count Ticks to wait
  */
static void LoopDelay(uint16_t loop_count)
{
    for (volatile int j = 0; j < loop_count; j++)
        ;
}

/**
  * @brief Data output to the HC595 8-Bit Shift Register
  * 
  * @param data Data byte
  */
static void OutData(uint8_t data)
{
    GPIO_WriteLow(STCP_PORT, STCP_PIN);

    LoopDelay(DELAY595);

    for (int i = 0; i < 8; i++)
    {
        GPIO_WriteLow(SHCP_PORT, SHCP_PIN);

        LoopDelay(DELAY595);

        if (data & 0x80)
        {
            GPIO_WriteHigh(D0_PORT, D0_PIN);
        }
        else
        {
            GPIO_WriteLow(D0_PORT, D0_PIN);
        }

        data <<= 1;

        LoopDelay(DELAY595);

        GPIO_WriteHigh(SHCP_PORT, SHCP_PIN);

        LoopDelay(DELAY595);
    }
    GPIO_WriteHigh(STCP_PORT, STCP_PIN);
    LoopDelay(DELAY595);
}

/**
  * @brief Initialise the Independent Watchdog (IWDG)
  */
static void InitialiseIWDG()
{
    IWDG_KR = 0xcc;   //  Start the independent watchdog.
    IWDG_KR = 0x55;   //  Allow the IWDG registers to be programmed.
    IWDG_PR = 0xFF;   //  Prescaler is 2 => each count is 250uS
    IWDG_RLR = 0xFF;  //  Reload counter.
    IWDG_KR = 0xaa;   //  Reset the counter.
}

/**
  * @brief Start timer capture
  */
void CaptureTimerStart(void)
{
    TIM3_DeInit();

    TIM3_TimeBaseInit(TIM3_PRESCALER_2, 0xFFFF);  //8 MHz

    //Channel 1, divider 8, rising polarity
    TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING,  //PD2 CN4.7
                TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0);

    //Enable 1st channel timer 1 high level clearing
    //TIM3->SMCR = 0x54;

    TIM3_ClearFlag(TIM3_FLAG_CC1);

    TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
    TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);

    TIM3_Cmd(ENABLE);
}

/*---------------------------------------------------------------------------*/
/*----------------------  PUBLIC FUNCTIONS  --------------------------------*/
/*---------------------------------------------------------------------------*/

/**
  * @brief  Firmware entry point, main loop
  */
void main(void)
{
    volatile uint32_t time_start = 0, time_last_pr = 0, time_lb = 0;
    volatile uint8_t tmp = 0;
    speed_modes_t speed = IDLE;
    volatile int periods_counter = 0;  //Used for debug purposes

    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOD);
    GPIO_DeInit(GPIOE);
    GPIO_DeInit(GPIOG);

    CLK_Config();
    TIM1_Config();
    InitialiseIWDG();
    CaptureTimerStart();
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz */
    //TIM2_TimeBaseInit(TIM1_PRESCALER_128, 124);

    TIM2_TimeBaseInit(TIM2_PRESCALER_1, PWM_PRES);

    /* Channel 1 PWM configuration */
    //TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE,PWM_NULL, TIM2_OCPOLARITY_LOW );
    //TIM2_OC1PreloadConfig(ENABLE);

    /* Channel 2 PWM configuration */
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, 0, TIM2_OCPOLARITY_LOW);
    TIM2_OC2PreloadConfig(ENABLE);

    /* Channel 3 PWM configuration */
    //TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE,PWM_NULL, TIM2_OCPOLARITY_LOW );
    //TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_SetCompare2(0);
    TIM2_Cmd(ENABLE);

    //----------- GPIO configuration --------------------------
#ifndef LED_WITH_SHIFTREGISTER
    GPIO_WriteLow(L1_PORT, L1_PIN);
    GPIO_WriteLow(L2_PORT, L2_PIN);
    GPIO_WriteLow(L3_PORT, L3_PIN);
#endif

    GPIO_Init(PROXIMITY_OUT_PORT, PROXIMITY_OUT_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    GPIO_Init(SHCP_PORT, SHCP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(STCP_PORT, STCP_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(D0_PORT, D0_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

    GPIO_WriteLow(SHCP_PORT, SHCP_PIN);
    GPIO_WriteLow(STCP_PORT, STCP_PIN);
    GPIO_WriteLow(D0_PORT, D0_PIN);

    GPIO_Init(B1_PORT, B1_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(B2_PORT, B2_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(B3_PORT, B3_PIN, GPIO_MODE_IN_PU_NO_IT);

    OutData((LED_NEGATIVE << B3_LED) | (LED_POSITIVE << B2_LED) | (LED_POSITIVE << B1_LED));
    //----
    GPIO_Init(FAST_SWITCH_PORT, FAST_SWITCH_PIN, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(GPIOD, GPIO_PIN_6);

    //------------- ADC configuration ------------------------

    /* NOTE: Uncomment to use analog proximity */
    //    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);//CN3-6
    //    ADC1_DeInit();
    //    ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
    //    ADC1_CHANNEL_4,
    //    ADC1_PRESSEL_FCPU_D2,
    //    ADC1_EXTTRIG_TIM,
    //    DISABLE,
    //    ADC1_ALIGN_RIGHT,
    //    ADC1_SCHMITTTRIG_CHANNEL4,
    //    DISABLE);
    //    ADC1_Cmd(ENABLE);
    //    ADC1_StartConversion();
    ADC1_Config();

    //------------ I2C configuration -------------------------

    GPIO_WriteHigh(GPIOD, GPIO_PIN_0);

#ifdef HARDWARE_I2C
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
#endif

    // Initialize APDS-9960 (configure I2C and initial values)
    if (apds.init())
    {
        //initialization complete;
    }
    else
    {
        assert_failed(__FILE__, __LINE__);
    }

    // Adjust the Proximity sensor gain
    if (!apds.setProximityGain(PGAIN_8X))
    {
        //Something went wrong trying to set PGAIN
        assert_failed(__FILE__, __LINE__);
    }

    apds.setMode(POWER, ON);
    tmp = apds.getMode();
    // Start running the APDS-9960 proximity sensor (no interrupts)
    if (apds.enableProximitySensor(false))
    {
        //Proximity sensor is now running
    }
    else
    {
        assert_failed(__FILE__, __LINE__);
        //Something went wrong during sensor init
    }
    apds.clearProximityInt();
    apds.clearAmbientLightInt();

    //------------ Main loop -------------------------

    while (1)
    {
        if (GPIO_ReadInputPin(FAST_SWITCH_PORT, FAST_SWITCH_PIN) == RESET)
        {
            time_to_run = DEFAULT_RUN_TIME;
        }
        else
        {
            time_to_run = DEFAULT_RUN_TIME * RUN_TIME_MULTIPLICATION;
        }
        //----------------------------------------------
        Buttns[0] = GPIO_ReadInputPin(B1_PORT, B1_PIN) == RESET ? 1 : 0;
        Buttns[1] = GPIO_ReadInputPin(B2_PORT, B2_PIN) == RESET ? 1 : 0;
        Buttns[2] = GPIO_ReadInputPin(B3_PORT, B3_PIN) == RESET ? 1 : 0;

        if ((Buttns[0] != old_Buttns[0]) && (SysTick - time_lb) > BUTTON_DEBOUNCE_TIME)
        {
            if (Buttns[0])
            {
                if (speed != FIRST_MODE)
                {
                    if (!speed) time_start = SysTick;
                    speed = FIRST_MODE;
                }
                else
                    speed = IDLE;
            }
            time_lb = SysTick;
            old_Buttns[0] = Buttns[0];
        }

        if ((Buttns[1] != old_Buttns[1]) && (SysTick - time_lb) > BUTTON_DEBOUNCE_TIME)
        {
            if (Buttns[1])
            {
                if (speed != SECOND_MODE)
                {
                    if (!speed) time_start = SysTick;
                    speed = SECOND_MODE;
                }
                else
                    speed = IDLE;
            }
            time_lb = SysTick;
            old_Buttns[1] = Buttns[1];
        }

        if ((Buttns[2] != old_Buttns[2]) && (SysTick - time_lb) > BUTTON_DEBOUNCE_TIME)
        {
            if (Buttns[2])
            {
                if (speed != THIRD_MODE)
                {
                    if (!speed) time_start = SysTick;
                    speed = THIRD_MODE;
                }
                else
                    speed = IDLE;
            }
            time_lb = SysTick;
            old_Buttns[2] = Buttns[2];
        }

        /* Stop the motor if timeout */
        if ((SysTick - time_start) > time_to_run && speed)
        {
            speed = IDLE;
        }

        /* Read the proximity threshold (variable resistor) */
        if ((SysTick - time_last_pr) > SENSOR_ACQUIRE_PERIOD)
        {
            proximity_threshold = GetADCvalue(9) / (PROXIMITY_THRESHOLD_PWM_CONSTANT / PWM_PRES);
            if (!aproximity_thresholds.readProximity(proximity_data))
            {
                periods_counter++;
                proximity_data = 0xFF;
            }
            else
            {
                if (proximity_data < proximity_threshold)
                {
                    speed = IDLE;
                    GPIO_WriteLow(PROXIMITY_OUT_PORT, PROXIMITY_OUT_PIN);

                    TIM2_SetCompare2(0);
                    OutData((LED_NEGATIVE << B3_LED) | (LED_POSITIVE << B2_LED) | (LED_POSITIVE << B1_LED));
                }
                else
                {
                    GPIO_WriteHigh(PROXIMITY_OUT_PORT, PROXIMITY_OUT_PIN);
                }
            }
            time_last_pr = SysTick;
        }
        //----------------------------------------------
        if (proximity_data >= proximity_threshold)
        {
            switch (speed)
            {
                case FIRST_MODE:
#ifndef LED_WITH_SHIFTREGISTER
                    GPIO_WriteLow(L2_PORT, L2_PIN);
                    GPIO_WriteLow(L3_PORT, L3_PIN);
#endif
                    TIM2_SetCompare2(GetADCvalue(0) / (PROXIMITY_THRESHOLD_PWM_CONSTANT / PWM_PRES));

                    if (((SysTick - time_start) / 100) % 10 > 5)
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteHigh(L1_PORT, L1_PIN);
#else
                        OutData(LED_NEGATIVE << B1_LED);
#endif
                    }
                    else
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteLow(L1_PORT, L1_PIN);
#else
                        OutData(LED_POSITIVE << B1_LED);
#endif
                    }

                    break;

                case SECOND_MODE:
#ifndef LED_WITH_SHIFTREGISTER
                    GPIO_WriteLow(L1_PORT, L1_PIN);
                    GPIO_WriteLow(L3_PORT, L3_PIN);
#endif
                    TIM2_SetCompare2(GetADCvalue(2) / (PROXIMITY_THRESHOLD_PWM_CONSTANT / PWM_PRES));
                    if (((SysTick - time_start) / 100) % 10 > 5)
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteHigh(L2_PORT, L2_PIN);
#else
                        OutData(LED_NEGATIVE << B2_LED);
#endif
                    }
                    else
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteLow(L2_PORT, L2_PIN);
#else
                        OutData(LED_POSITIVE << B2_LED);
#endif
                    }
                    break;

                case THIRD_MODE:
#ifndef LED_WITH_SHIFTREGISTER
                    GPIO_WriteLow(L1_PORT, L1_PIN);
                    GPIO_WriteLow(L2_PORT, L2_PIN);
#endif
                    //need=0...1024
                    //uCaptureValue=0xFFFF...0
                    //pwm=0..PWM_PRES
                    //N (rev/min) = 1/((65535-813*64)/8e6*16*6)*60

                    float K = 1;
                    float M = 0;  //can be 0.15;

                    need = GetADCvalue(4);
                    get = (0xFFFF - uCaptureValue) / 64 * K;
                    pwm = ((float)need + ((float)need - (float)get) * M) / (PROXIMITY_THRESHOLD_PWM_CONSTANT / (float)PWM_PRES);
                    TIM2_SetCompare2(pwm);
                    if (((SysTick - time_start) / 100) % 10 > 5)
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteHigh(L3_PORT, L3_PIN);
#else
                        OutData(LED_POSITIVE << B3_LED);
#endif
                    }
                    else
                    {
#ifndef LED_WITH_SHIFTREGISTER
                        GPIO_WriteLow(L3_PORT, L3_PIN);
#else
                        OutData(LED_NEGATIVE << B3_LED);
#endif
                    }
                    break;

                default:
                    speed = IDLE;
                    TIM2_SetCompare2(0);
#ifndef LED_WITH_SHIFTREGISTER
                    GPIO_WriteHigh(L1_PORT, L1_PIN);
                    GPIO_WriteHigh(L2_PORT, L2_PIN);
                    GPIO_WriteHigh(L3_PORT, L3_PIN);
#else
                    OutData((LED_NEGATIVE << B3_LED) | (LED_POSITIVE << B2_LED) | (LED_POSITIVE << B1_LED));
#endif
                    break;
            }
        }

        /* Reload IWDG */
        IWDG_KR = IWDG_RELOAD_VALUE;
    };
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval 
  * None
  */
void assert_failed(u8* file, u32 line)
{
    while (1)
    {
    }
}
#endif
