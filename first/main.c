#include <__cross_studio_io.h>
#include <stm32f4xx.h>

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

void main(void)
{
        init_systick();
        init_led();
        while(1) {
                 GPIOA->ODR ^= GPIO_BSRR_BS_5;
                 delay_ms(100);
        }
}

void SysTick_Handler(void)
{
        if (timeout != 0) {
                --timeout;
        }
}
