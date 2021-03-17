#include "HAL_conf.h"
#include "HAL_device.h"
#include "stdio.h"
#include "FreeRTOS.h"  
#include "task.h" 
#include "queue.h" 
#include "list.h"  
#include "portable.h"  
#include "FreeRTOSConfig.h" 


static void prvSetupHardware( void );

void  vLED_1_Task( void *pvParameters ) 
{  
    while( 1 ) {  
        GPIO_ResetBits( GPIOA, GPIO_Pin_8 );
        printf("LED1 ON\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS ); 
        GPIO_SetBits( GPIOA, GPIO_Pin_8);
        printf("LED1 OFF\r\n");
        vTaskDelay( 2000 / portTICK_RATE_MS ); 
    }  
}

void  vLED_2_Task( void *pvParameters ) 
{  
    while( 1 ) {  
        GPIO_SetBits( GPIOA, GPIO_Pin_11);
        printf("LED2 OFF\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS );
        GPIO_ResetBits( GPIOA, GPIO_Pin_11);
        printf("LED2 ON\r\n");
        vTaskDelay( 1000 / portTICK_RATE_MS );  
    }  
} 


int main(void)
{
	prvSetupHardware();
    				    	    
    
    printf("FreeRTOS开始运行\r\n");
    xTaskCreate( vLED_1_Task, "LED1",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1 ,NULL);        
    xTaskCreate( vLED_2_Task, "LED2",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+2 ,NULL);
    
	vTaskStartScheduler();
	
	for( ;; );
}


/********************************************************************************************************
**函数信息 ：void TIM3_PWM_Init(u16 arr,u16 psc)                       
**功能描述 ：TIM3 PWM初始化
**输入参数 ：u16 arr,u16 psc
**输出参数 ：
**常用函数 ：
********************************************************************************************************/
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
    
    //设置该引脚为复用推挽输出功能,输出TIM3 CH1  的PWM脉冲波形
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;					              
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 				          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2);
    
    /*设置在下一个更新事件装入活动的自动重装载寄存器周期的值*/
    TIM_TimeBaseStructure.TIM_Period = arr; 
    /*设置用来作为TIMx时钟频率除数的预分频值  不分频*/    
    TIM_TimeBaseStructure.TIM_Prescaler =psc; 		
    /*设置时钟分割:TDTS = Tck_tim*/    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	
    /*TIM向上计数模式*/    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    /*根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位*/    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 				            
       
    /*选择定时器模式:TIM脉冲宽度调制模式1*/       
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 	
    /*比较输出使能*/    
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    /*设置待装入捕获比较寄存器的脉冲值*/    
    TIM_OCInitStructure.TIM_Pulse = 0; 		
    /*输出极性:TIM输出比较极性高*/    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			        
    
    /*根据TIM_OCInitStruct中指定的参数初始化外设TIMx*/
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  		
    /*使能TIMx在CCR1上的预装载寄存器*/    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  	               			
           
    /*使能TIMx在ARR上的预装载寄存器*/ 
    TIM_ARRPreloadConfig(TIM3, ENABLE); 				               	        
 
    /*使能TIMx外设*/ 
    TIM_Cmd(TIM3, ENABLE);  							                        
}


/********************************************************************************************************
**函数信息 ：void uart_initwBaudRate(u32 bound)     
**功能描述 ：UART初始化
**输入参数 ：bound
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void uart_initwBaudRate(u32 bound)
{
    /*GPIO端口设置*/
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    /*UART1_TX   GPIOA.9*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /*复用推挽输出*/    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                     
    GPIO_Init(GPIOA, &GPIO_InitStructure);				                        
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
    
    /*UART1_RX  GPIOA.10初始化*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    /*浮空输入*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	                 
    GPIO_Init(GPIOA, &GPIO_InitStructure);				                     
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
    
    /*UART 初始化设置*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);	                   

    /*串口波特率*/
    UART_InitStructure.UART_BaudRate = bound;
    /*字长为8位数据格式*/    
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    /*一个停止位*/    
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    /*无奇偶校验位*/    
    UART_InitStructure.UART_Parity = UART_Parity_No;
    /*无硬件数据流控制*/    
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    /*收发模式*/
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	                
    /*初始化串口1*/
    UART_Init(UART1, &UART_InitStructure); 	
    /*开启串口接受中断*/    
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);
    /*使能串口1*/    
    UART_Cmd(UART1, ENABLE);                    			                    
}


/********************************************************************************************************
**函数信息 ：void UartSendByte(u8 dat)      
**功能描述 ：UART发送数据
**输入参数 ：dat
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData( UART1, dat);
    while(!UART_GetFlagStatus(UART1,UART_FLAG_TXEPT));
}


/********************************************************************************************************
**函数信息 ：void UartSendGroup(u8* buf,u16 len)     
**功能描述 ：UART发送数据
**输入参数 ：buf,len
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void UartSendGroup(u8* buf,u16 len)
{
    while(len--)
        UartSendByte(*buf++);
}

int fputc(int ch, FILE *f)  
{  
    /* 写一个字节到USART1 */  
    UART_SendData(UART1, (uint8_t) ch);  
    /* 等待发送结束 */  
    while (UART_GetFlagStatus(UART1, UART_FLAG_TXEPT) == RESET)  
    {}  
    return ch;  
}

/********************************************************************************************************
**函数信息 ：void LED_Init(void)     
**功能描述 ：LED管脚初使化输出
**输入参数 ：
**输出参数 ：
**    备注 ：
********************************************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;       
    RCC_AHB1PeriphClockCmd(  RCC_AHB1Periph_GPIOA,  
                             ENABLE
                           );      
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8|GPIO_Pin_11;     
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_Init( GPIOA, &GPIO_InitStructure );    
    GPIO_SetBits( GPIOA, GPIO_Pin_8 );
    GPIO_SetBits( GPIOA, GPIO_Pin_9 );
}


/********************************************************************************************************
**函数信息 ：static void prvSetupHardware(void)     
**功能描述 ：硬件初使化
**输入参数 ：
**输出参数 ：
**    备注 ：也有人声明为static void bspInit(void)
********************************************************************************************************/
static void prvSetupHardware( void ) 
{
    /*
     * MM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
     * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
     * 都统一用这个优先级分组，千万不要再分组，切忌。
     */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    LED_Init();
    uart_initwBaudRate(115200);
    /*1KHZ周期*/
    TIM3_PWM_Init(1000-1, SystemCoreClock/1000000-1);   	
    /*设置占空比*/    
    TIM_SetCompare1(TIM3, 200);								
}
