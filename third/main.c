#include <__cross_studio_io.h>
#include <stm32f4xx.h>
#include <math.h>

__IO uint32_t timeout = 0;
void delay_ms(__IO uint32_t t)
{
    timeout = t;
    while (timeout != 0);
}

/*
User LD2: the green LED is a user LED connected to Arduino signal D13 corresponding to
MCU I/O PA5 (pin 21) or PB13 (pin 34) depending on the STM32 target. Please refer to
Table 10 to Table 19.
*/
void init_led(void)
{
    // GPIOA and PA5 LED, AHB1
    // Before using a peripheral you have to enable its clock in the
    // RCC_AHBxENR or RCC_APBxENR register.
    // 6.3.9
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Errata: Delay after an RCC peripheral clock enabling
    __NOP(); __NOP();
    // PA5
    // 01: General Purpose Output Mode
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    // High Speed
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR5_1;
    // Pull up
    //GPIOE->PUPDR |= GPIO_PUPDR_PUPDR9_0;
    // Drive Pin High
    GPIOA->BSRR |= GPIO_BSRR_BS_5;
}

void init_systick(void)
{
    const uint32_t HSI_VALUE = 16000000;
    uint32_t temp = 0;

    temp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
    if (temp) {
        // This is bad! If we aren't zero, we are running slow.
        debug_printf("HPRE Prescaler isn't 0!");
    }
    // 1ms
    SysTick->LOAD = (HSI_VALUE / 1000) - 1;
    NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0;
    // Use AHB Clock, Assert on zero, Enable
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_TICKINT_Msk  |
        SysTick_CTRL_ENABLE_Msk;
}

uint16_t get_temperature_raw(void)
{
    // Wait for end of conversion
    while ((ADC1->SR & ADC_SR_EOC) != ADC_SR_EOC);
    return (uint16_t)(ADC1->DR & 0xFFF);
}

/*
TS_CAL1 TS ADC raw data acquired at temperature of 30 �C, V DDA = 3.3 V 0x1FFF 7A2C - 0x1FFF 7A2D
TS_CAL2 TS ADC raw data acquired at temperature of 110 �C, V DDA = 3.3 V 0x1FFF 7A2E - 0x1FFF 7A2F

V 25 (1) Voltage at 25 �C - 0.76 - V
*/
#define TS_CAL1 (*(uint16_t*)(0x1FFF7A2C))
#define TS_CAL2 (*(uint16_t*)(0x1FFF7A2E))
#define mV_per_ADC_Step (0.8056640625)
#define AVG_SLOPE (float)((TS_CAL1 - TS_CAL2)/80.0)
#define Steps_at_V25 (760/mV_per_ADC_Step)

void init_temperature(void)
{
    /*
    3.  Select ADC1_IN16 or ADC1_IN18 input channel.
    4.  Select a sampling time greater than the minimum sampling time specified in the
    datasheet.
    5.  Set the TSVREFE bit in the ADC_CCR register to wake up the temperature sensor
    from power down mode
    6.  Start the ADC conversion by setting the SWSTART bit (or by external trigger)
    7.  Read the resulting V SENSE data in the ADC data register
    8.  Calculate the temperature using the following formula:
    Temperature (in �C) = {(V SENSE � V 25 ) / Avg_Slope} + 25
    Where:
    � V 25 = V SENSE value for 25� C
    � Avg_Slope = average slope of the temperature vs. V SENSE curve (given in mV/�C
    or �V/�C)
    Refer to the datasheet�s electrical characteristics section for the actual values of V 25
    and Avg_Slope.
    */

    // Enable the ADC Periph clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __NOP(); __NOP();
    // Enable the Temperature Sensor
    ADC->CCR |= ADC_CCR_TSVREFE;
    // Turn on continuous mode
    ADC1->CR2 |= ADC_CR2_CONT;

    // Set Sample Rate to 56 cycles for channel 18
    ADC1->SMPR1 |=ADC_SMPR1_SMP18_0 | ADC_SMPR1_SMP18_1;

    // Set Channel 18 as rank 1 (5 bits wide per register * (rank - 1))
    ADC1->SQR3 |= 18 << (5 * (1-1));

    // Turn it on
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    debug_printf("TS_CAL1 (30C) = %d (%.1fmV)\n", TS_CAL1, TS_CAL1*mV_per_ADC_Step);
    debug_printf("TS_CAL2 (110C)= %d (%.1fmV)\n", TS_CAL2, TS_CAL2*mV_per_ADC_Step);
}

void init_rtc(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    // Enable the RTC LSI
    RCC->CSR |= RCC_CSR_LSION;
    // Wait for the LSI to be ready
    while ((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY);
    /*
    The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control
    register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
    write-protected and the DBP bit in the PWR power control register (PWR_CR) has to be set
    before these can be modified.
    */
    PWR->CR |= PWR_CR_DBP;
    // Reset the Backup Domain Registers - Datasheet is a bit misleading... The registers can
    // only be set AFTER a reset is done not WHILE its being reset.
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~(RCC_BDCR_BDRST);
    // Set LSI and Turn on the RTC Clock
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;
    RCC->BDCR |= RCC_BDCR_RTCEN;
    /*
    After backup domain reset, all the RTC registers are write-protected. Writing to the RTC
    registers is enabled by writing a key into the Write Protection register, RTC_WPR.
    The following steps are required to unlock the write protection on all the RTC registers
    except for RTC_ISR[13:8], RTC_TAFCR, and RTC_BKPxR.
    1. Write �0xCA� into the RTC_WPR register.
    2.  Write �0x53� into the RTC_WPR register.
    */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    /* Enter Calendar Initialization */
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) != RTC_ISR_INITF);
    /*
    The asynchronous prescaler division factor is set to 128, and the synchronous division
    factor to 256, to obtain an internal clock frequency of 1 Hz (ck_spre) with an LSE frequency
    of 32.768 kHz.

    STM32F411x min/typ/max 17/32/47 kHz
    17.3.1  Clock and prescalers
    */
    // ck_spre frequency = ck_apre frequency/(PREDIV_S+1)
    // 32.768 / (128+1) = 0.25 or ~4Hz
    // ck_apre frequency = RTCCLK frequency/(PREDIV_A+1)
    // 32.768 / (256+1) =
    RTC->PRER = 0xFF;
    RTC->PRER |= 0x7F;
    // Set the time BCD Format
    RTC->TR = 0b100001110000100000000;
    // Set the Date BCD Format
    RTC->DR = 0b101001111000011000100101;
    /* Leave Calendar Initialization */
    RTC->ISR &= ~(RTC_ISR_INIT);
}

void main(void)
{
    init_systick();
    init_led();
    init_temperature();
    init_rtc();
    uint16_t raw[10];
    uint8_t raw_index = 0;
    while(1) {
        // Toggle the LED
        GPIOA->ODR ^= GPIO_BSRR_BS_5;
        delay_ms(100);
        // Average and Display the Temperature every 10 samples.
        if (raw_index == 10) {
            float average = 0;
            for (int i=0; i < 10; ++i) {
                average += raw[i];
            }
            average /= 10;
            average = floor(average + 0.5);
            // Formula Taken from Datasheet
            float cel = ((((Steps_at_V25-average)/AVG_SLOPE))+25);
            cel = floor(cel + 0.5);
            debug_printf("Temp: %.1fC/%.1fF\t%.1f              \r", cel, (cel*1.8)+32.0, average);
            raw_index = 0;
        }
        raw[raw_index++] = get_temperature_raw();
    }
}

void SysTick_Handler(void)
{
    if (timeout != 0) {
        --timeout;
    }
}
