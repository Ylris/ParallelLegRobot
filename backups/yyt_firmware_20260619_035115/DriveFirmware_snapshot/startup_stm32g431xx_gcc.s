.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global g_pfnVectors
.global Reset_Handler

.word _sidata

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  ldr r0, =_estack
  mov sp, r0

  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
CopyData:
  cmp r0, r1
  bcc CopyDataLoop
  b ZeroBss
CopyDataLoop:
  ldr r3, [r2]
  str r3, [r0]
  adds r2, r2, #4
  adds r0, r0, #4
  b CopyData

ZeroBss:
  ldr r0, =_sbss
  ldr r1, =_ebss
  movs r2, #0
ZeroBssLoop:
  cmp r0, r1
  bcc ZeroBssStore
  b CallMain
ZeroBssStore:
  str r2, [r0]
  adds r0, r0, #4
  b ZeroBssLoop

CallMain:
  bl SystemInit
  bl __libc_init_array
  bl main
LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

.section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
.size Default_Handler, .-Default_Handler

.macro def_irq name
  .weak \name
  .thumb_set \name, Default_Handler
.endm

def_irq NMI_Handler
def_irq HardFault_Handler
def_irq MemManage_Handler
def_irq BusFault_Handler
def_irq UsageFault_Handler
def_irq SVC_Handler
def_irq DebugMon_Handler
def_irq PendSV_Handler
def_irq SysTick_Handler
def_irq WWDG_IRQHandler
def_irq PVD_PVM_IRQHandler
def_irq RTC_TAMP_LSECSS_IRQHandler
def_irq RTC_WKUP_IRQHandler
def_irq FLASH_IRQHandler
def_irq RCC_IRQHandler
def_irq EXTI0_IRQHandler
def_irq EXTI1_IRQHandler
def_irq EXTI2_IRQHandler
def_irq EXTI3_IRQHandler
def_irq EXTI4_IRQHandler
def_irq DMA1_Channel1_IRQHandler
def_irq DMA1_Channel2_IRQHandler
def_irq DMA1_Channel3_IRQHandler
def_irq DMA1_Channel4_IRQHandler
def_irq DMA1_Channel5_IRQHandler
def_irq DMA1_Channel6_IRQHandler
def_irq ADC1_2_IRQHandler
def_irq USB_HP_IRQHandler
def_irq USB_LP_IRQHandler
def_irq FDCAN1_IT0_IRQHandler
def_irq FDCAN1_IT1_IRQHandler
def_irq EXTI9_5_IRQHandler
def_irq TIM1_BRK_TIM15_IRQHandler
def_irq TIM1_UP_TIM16_IRQHandler
def_irq TIM1_TRG_COM_TIM17_IRQHandler
def_irq TIM1_CC_IRQHandler
def_irq TIM2_IRQHandler
def_irq TIM3_IRQHandler
def_irq TIM4_IRQHandler
def_irq I2C1_EV_IRQHandler
def_irq I2C1_ER_IRQHandler
def_irq I2C2_EV_IRQHandler
def_irq I2C2_ER_IRQHandler
def_irq SPI1_IRQHandler
def_irq SPI2_IRQHandler
def_irq USART1_IRQHandler
def_irq USART2_IRQHandler
def_irq USART3_IRQHandler
def_irq EXTI15_10_IRQHandler
def_irq RTC_Alarm_IRQHandler
def_irq USBWakeUp_IRQHandler
def_irq TIM8_BRK_IRQHandler
def_irq TIM8_UP_IRQHandler
def_irq TIM8_TRG_COM_IRQHandler
def_irq TIM8_CC_IRQHandler
def_irq LPTIM1_IRQHandler
def_irq SPI3_IRQHandler
def_irq UART4_IRQHandler
def_irq TIM6_DAC_IRQHandler
def_irq TIM7_IRQHandler
def_irq DMA2_Channel1_IRQHandler
def_irq DMA2_Channel2_IRQHandler
def_irq DMA2_Channel3_IRQHandler
def_irq DMA2_Channel4_IRQHandler
def_irq DMA2_Channel5_IRQHandler
def_irq UCPD1_IRQHandler
def_irq COMP1_2_3_IRQHandler
def_irq COMP4_IRQHandler
def_irq CRS_IRQHandler
def_irq SAI1_IRQHandler
def_irq FPU_IRQHandler
def_irq RNG_IRQHandler
def_irq LPUART1_IRQHandler
def_irq I2C3_EV_IRQHandler
def_irq I2C3_ER_IRQHandler
def_irq DMAMUX_OVR_IRQHandler
def_irq DMA2_Channel6_IRQHandler
def_irq CORDIC_IRQHandler
def_irq FMAC_IRQHandler

.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors
g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_PVM_IRQHandler
  .word RTC_TAMP_LSECSS_IRQHandler
  .word RTC_WKUP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word DMA1_Channel6_IRQHandler
  .word 0
  .word ADC1_2_IRQHandler
  .word USB_HP_IRQHandler
  .word USB_LP_IRQHandler
  .word FDCAN1_IT0_IRQHandler
  .word FDCAN1_IT1_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_TIM15_IRQHandler
  .word TIM1_UP_TIM16_IRQHandler
  .word TIM1_TRG_COM_TIM17_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word USBWakeUp_IRQHandler
  .word TIM8_BRK_IRQHandler
  .word TIM8_UP_IRQHandler
  .word TIM8_TRG_COM_IRQHandler
  .word TIM8_CC_IRQHandler
  .word 0
  .word 0
  .word LPTIM1_IRQHandler
  .word 0
  .word SPI3_IRQHandler
  .word UART4_IRQHandler
  .word 0
  .word TIM6_DAC_IRQHandler
  .word TIM7_IRQHandler
  .word DMA2_Channel1_IRQHandler
  .word DMA2_Channel2_IRQHandler
  .word DMA2_Channel3_IRQHandler
  .word DMA2_Channel4_IRQHandler
  .word DMA2_Channel5_IRQHandler
  .word 0
  .word 0
  .word UCPD1_IRQHandler
  .word COMP1_2_3_IRQHandler
  .word COMP4_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word CRS_IRQHandler
  .word SAI1_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word FPU_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word RNG_IRQHandler
  .word LPUART1_IRQHandler
  .word I2C3_EV_IRQHandler
  .word I2C3_ER_IRQHandler
  .word DMAMUX_OVR_IRQHandler
  .word 0
  .word 0
  .word DMA2_Channel6_IRQHandler
  .word 0
  .word 0
  .word CORDIC_IRQHandler
  .word FMAC_IRQHandler
