;******************** (C) COPYRIGHT 2018 MindMotion ********************
; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>
Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB
				
				IMPORT xPortPendSVHandler		; insert for freeRTOS
				IMPORT xPortSysTickHandler		; insert for freeRTOS
				IMPORT vPortSVCHandler			; insert for freeRTOS

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; NMI Handler
                DCD     HardFault_Handler          ; Hard Fault Handler
                DCD     MemManage_Handler          ; MPU Fault Handler
                DCD     BusFault_Handler           ; Bus Fault Handler
                DCD     UsageFault_Handler         ; Usage Fault Handler
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                ;DCD     SVC_Handler                ; SVCall Handler
				DCD 	vPortSVCHandler			   ; change for freeRTOS
                DCD     DebugMon_Handler           ; Debug Monitor Handler
                DCD     0                          ; Reserved
                ;DCD     PendSV_Handler             ; PendSV Handler
                ;DCD     SysTick_Handler            ; SysTick Handler  ; External Interrupts   
				DCD		xPortPendSVHandler		   ; change for freeRTOS
				DCD		xPortSysTickHandler		   ; change for freeRTOS	
                DCD     WWDG_IRQHandler            ; Window Watchdog
                DCD     PVD_IRQHandler             ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler          ; Tamper
                DCD     RTC_IRQHandler             ; RTC
                DCD     FLASH_IRQHandler           ; Flash
                DCD     RCC_CRS_IRQHandler         ; RCC
                DCD     EXTI0_IRQHandler           ; EXTI Line 0
                DCD     EXTI1_IRQHandler           ; EXTI Line 1
                DCD     EXTI2_IRQHandler           ; EXTI Line 2
                DCD     EXTI3_IRQHandler           ; EXTI Line 3
                DCD     EXTI4_IRQHandler           ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler   ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler   ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler   ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler   ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler   ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler   ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler   ; DMA1 Channel 7
                DCD     ADC1_IRQHandler            ; ADC1
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     CAN1_RX_IRQHandler         ; CAN1_RX
                DCD     0                          ; Reserved
                DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
                DCD     TIM1_BRK_IRQHandler        ; TIM1 Break
                DCD     TIM1_UP_IRQHandler         ; TIM1 Update
                DCD     TIM1_TRG_COM_IRQHandler    ; TIM1 Trigger and Commutation
                DCD     TIM1_CC_IRQHandler         ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler            ; TIM2
                DCD     TIM3_IRQHandler            ; TIM3
                DCD     TIM4_IRQHandler            ; TIM4
                DCD     I2C1_IRQHandler            ; I2C1 Event
                DCD     0                          ; Reserved
                DCD     I2C2_IRQHandler            ; I2C2 Event
                DCD     0                          ; Reserved
                DCD     SPI1_IRQHandler            ; SPI1
                DCD     SPI2_IRQHandler            ; SPI2
                DCD     UART1_IRQHandler           ; UART1
                DCD     UART2_IRQHandler           ; UART2
                DCD     UART3_IRQHandler           ; UART3
                DCD     EXTI15_10_IRQHandler       ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler        ; RTC Alarm through EXTI Line 17
								DCD     USB_WKUP_IRQHandler       ; USB Wakeup from suspend
                DCD     TIM8_BRK_IRQHandler        ; TIM8 Break
                DCD     TIM8_UP_IRQHandler         ; TIM8 Update
                DCD     TIM8_TRG_COM_IRQHandler    ; TIM8 Trigger and Commutation
                DCD     TIM8_CC_IRQHandler         ; TIM8 Capture Compare
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     SDIO_IRQHandler            ; SDIO
								DCD     TIM5_IRQHandler            ; TIM5
								DCD     SPI3_IRQHandler            ; SPI3
								DCD     UART4_IRQHandler           ; UART4
                DCD     UART5_IRQHandler           ; UART5
								DCD     TIM6_IRQHandler            ; TIM6
								DCD     TIM7_IRQHandler            ; TIM7
								DCD     DMA2_Channel1_IRQHandler   ; DMA2 Channel 1
                DCD     DMA2_Channel2_IRQHandler   ; DMA2 Channel 2
                DCD     DMA2_Channel3_IRQHandler   ; DMA2 Channel 3
                DCD     DMA2_Channel4_IRQHandler   ; DMA2 Channel 4
                DCD     DMA2_Channel5_IRQHandler   ; DMA2 Channel 5
								DCD     ETHERNET_MAC_IRQHandler    ; Ethernet
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     COMP1_2_IRQHandler         ; COMP1,COMP2
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     USB_OTG_FS_IRQHandler      ; USB_FS
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     UART6_IRQHandler           ; UART6  
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     AES_IRQHandler             ; AES
								DCD     TRNG_IRQHandler            ; TRNG
								DCD     0                          ; Reserved
								DCD     UART7_IRQHandler           ; UART7
								DCD     UART8_IRQHandler           ; UART8
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								DCD     0                          ; Reserved
								
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
     IMPORT  __main
     IMPORT  SystemInit
                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler            [WEAK]
                EXPORT  PVD_IRQHandler             [WEAK]
                EXPORT  TAMPER_IRQHandler          [WEAK]
                EXPORT  RTC_IRQHandler             [WEAK]
                EXPORT  FLASH_IRQHandler           [WEAK]
                EXPORT  RCC_CRS_IRQHandler         [WEAK]
                EXPORT  EXTI0_IRQHandler           [WEAK]
                EXPORT  EXTI1_IRQHandler           [WEAK]
                EXPORT  EXTI2_IRQHandler           [WEAK]
                EXPORT  EXTI3_IRQHandler           [WEAK]
                EXPORT  EXTI4_IRQHandler           [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel2_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel3_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel4_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel5_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel6_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel7_IRQHandler   [WEAK]
                EXPORT  ADC1_IRQHandler            [WEAK]
                EXPORT  CAN1_RX_IRQHandler         [WEAK]	;CHEND
                EXPORT  EXTI9_5_IRQHandler         [WEAK]
                EXPORT  TIM1_BRK_IRQHandler        [WEAK]
                EXPORT  TIM1_UP_IRQHandler         [WEAK]
                EXPORT  TIM1_TRG_COM_IRQHandler    [WEAK]
                EXPORT  TIM1_CC_IRQHandler         [WEAK]
                EXPORT  TIM2_IRQHandler            [WEAK]
                EXPORT  TIM3_IRQHandler            [WEAK]
                EXPORT  TIM4_IRQHandler            [WEAK]
                EXPORT  I2C1_IRQHandler            [WEAK]
                EXPORT  I2C2_IRQHandler            [WEAK]
                EXPORT  SPI1_IRQHandler            [WEAK]
                EXPORT  SPI2_IRQHandler            [WEAK]
                EXPORT  UART1_IRQHandler           [WEAK]
                EXPORT  UART2_IRQHandler           [WEAK]
                EXPORT  UART3_IRQHandler           [WEAK]
                EXPORT  EXTI15_10_IRQHandler       [WEAK]
                EXPORT  RTCAlarm_IRQHandler        [WEAK]
                EXPORT  USB_WKUP_IRQHandler        [WEAK]	;CHEND
								EXPORT  TIM8_BRK_IRQHandler        [WEAK]
                EXPORT  TIM8_UP_IRQHandler         [WEAK]
                EXPORT  TIM8_TRG_COM_IRQHandler    [WEAK]
                EXPORT  TIM8_CC_IRQHandler         [WEAK]
								EXPORT  SDIO_IRQHandler            [WEAK]
								EXPORT  TIM5_IRQHandler            [WEAK]
								EXPORT  SPI3_IRQHandler            [WEAK]
								EXPORT  UART4_IRQHandler           [WEAK]
								EXPORT  UART5_IRQHandler           [WEAK]
								EXPORT  TIM6_IRQHandler            [WEAK]
								EXPORT  TIM7_IRQHandler            [WEAK]
								EXPORT  DMA2_Channel1_IRQHandler   [WEAK]
                EXPORT  DMA2_Channel2_IRQHandler   [WEAK]
                EXPORT  DMA2_Channel3_IRQHandler   [WEAK]
                EXPORT  DMA2_Channel4_IRQHandler   [WEAK]
                EXPORT  DMA2_Channel5_IRQHandler   [WEAK]
								EXPORT  ETHERNET_MAC_IRQHandler    [WEAK]	;CHEND
								EXPORT  COMP1_2_IRQHandler         [WEAK]
								EXPORT  USB_OTG_FS_IRQHandler      		 [WEAK]	;CHEND
								EXPORT  UART6_IRQHandler           [WEAK]
								EXPORT  AES_IRQHandler             [WEAK]
								EXPORT  TRNG_IRQHandler            [WEAK]
								EXPORT  UART7_IRQHandler           [WEAK]
								EXPORT  UART8_IRQHandler           [WEAK]
				
WWDG_IRQHandler         
PVD_IRQHandler          
TAMPER_IRQHandler       
RTC_IRQHandler          
FLASH_IRQHandler        
RCC_CRS_IRQHandler      
EXTI0_IRQHandler        
EXTI1_IRQHandler        
EXTI2_IRQHandler        
EXTI3_IRQHandler        
EXTI4_IRQHandler        
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_IRQHandler         
CAN1_RX_IRQHandler      
EXTI9_5_IRQHandler      
TIM1_BRK_IRQHandler     
TIM1_UP_IRQHandler      
TIM1_TRG_COM_IRQHandler 
TIM1_CC_IRQHandler      
TIM2_IRQHandler         
TIM3_IRQHandler         
TIM4_IRQHandler         
I2C1_IRQHandler         
I2C2_IRQHandler         
SPI1_IRQHandler         
SPI2_IRQHandler         
UART1_IRQHandler        
UART2_IRQHandler        
UART3_IRQHandler        
EXTI15_10_IRQHandler    
RTCAlarm_IRQHandler     
USB_WKUP_IRQHandler     
TIM8_BRK_IRQHandler     
TIM8_UP_IRQHandler      
TIM8_TRG_COM_IRQHandler 
TIM8_CC_IRQHandler      
SDIO_IRQHandler         
TIM5_IRQHandler         
SPI3_IRQHandler         
UART4_IRQHandler        
UART5_IRQHandler        
TIM6_IRQHandler         
TIM7_IRQHandler         
DMA2_Channel1_IRQHandler
DMA2_Channel2_IRQHandler
DMA2_Channel3_IRQHandler
DMA2_Channel4_IRQHandler
DMA2_Channel5_IRQHandler
ETHERNET_MAC_IRQHandler 
COMP1_2_IRQHandler      
USB_OTG_FS_IRQHandler      	
UART6_IRQHandler        
AES_IRQHandler          
TRNG_IRQHandler         
UART7_IRQHandler        
UART8_IRQHandler        

                             
                B       .    
                             
                ENDP         
                             
                ALIGN        
                             
;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB           
                             
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                             
                 ELSE        
                             
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                             
__user_initial_stackheap     
                             
                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR  
                             
                 ALIGN       
                             
                 ENDIF       
                             
                 END         
                             
;******************** (C) COPYRIGHT 2018 MindMotion ********************
                             