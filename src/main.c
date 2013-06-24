#include "stm32f4xx_conf.h"

#include "sys.h"
#include "leds.h"
#include "stepper.h"
#include "inputs.h"

#include <stdio.h>
#include <math.h>


void EXTI0_IRQHandler(void)
{
    TIM4->CCR1 = 0x001;
    TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = TIM4->CCR1;

    TIM2->ARR -= (TIM2->ARR / 20);
    //freq_request ++;// (freq_request /20);

    //GPIOD->ODR ^= (1 << 13);

    //EXTI->PR |= EXTI_PR_PR0; // reset the interrupt
    EXTI_ClearITPendingBit(EXTI_Line0);
}


#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

//uint32_t tick = 0;
volatile uint32_t time_var1, time_var2;


void calculation_test();

int main(void)
{
    sys_init();
    leds_init();
    stepper_init();
    inputs_init();

    // ---------- SysTick timer -------- //
    if (SysTick_Config(SystemCoreClock / 1000)) {
            // Capture error
            while (1){};
    }

    // ------------- USB -------------- //
    USBD_Init(&USB_OTG_dev,
                USB_OTG_FS_CORE_ID,
                &USR_desc,
                &USBD_CDC_cb,
                &USR_cb);

    setbuf(stdout, NULL);

    while (1)
    {
        // should go idle...
        printf("test\n\r");
        Delay(500);
    }

}

/*
 * Called from systick handler
 */
void timing_handler() {
        if (time_var1) {
                time_var1--;
        }

        time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
        time_var1 = nCount;
        while(time_var1){};
}

