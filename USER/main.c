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
    				    	    
    
    printf("FreeRTOS��ʼ����\r\n");
    xTaskCreate( vLED_1_Task, "LED1",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+1 ,NULL);        
    xTaskCreate( vLED_2_Task, "LED2",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY+2 ,NULL);
    
	vTaskStartScheduler();
	
	for( ;; );
}


/********************************************************************************************************
**������Ϣ ��void TIM3_PWM_Init(u16 arr,u16 psc)                       
**�������� ��TIM3 PWM��ʼ��
**������� ��u16 arr,u16 psc
**������� ��
**���ú��� ��
********************************************************************************************************/
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
    
    //���ø�����Ϊ���������������,���TIM3 CH1  ��PWM���岨��
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;					              
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 				          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2);
    
    /*��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ*/
    TIM_TimeBaseStructure.TIM_Period = arr; 
    /*����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ*/    
    TIM_TimeBaseStructure.TIM_Prescaler =psc; 		
    /*����ʱ�ӷָ�:TDTS = Tck_tim*/    
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	
    /*TIM���ϼ���ģʽ*/    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    /*����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ*/    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 				            
       
    /*ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1*/       
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 	
    /*�Ƚ����ʹ��*/    
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    /*���ô�װ�벶��ȽϼĴ���������ֵ*/    
    TIM_OCInitStructure.TIM_Pulse = 0; 		
    /*�������:TIM����Ƚϼ��Ը�*/    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			        
    
    /*����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx*/
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  		
    /*ʹ��TIMx��CCR1�ϵ�Ԥװ�ؼĴ���*/    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  	               			
           
    /*ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���*/ 
    TIM_ARRPreloadConfig(TIM3, ENABLE); 				               	        
 
    /*ʹ��TIMx����*/ 
    TIM_Cmd(TIM3, ENABLE);  							                        
}


/********************************************************************************************************
**������Ϣ ��void uart_initwBaudRate(u32 bound)     
**�������� ��UART��ʼ��
**������� ��bound
**������� ��
**    ��ע ��
********************************************************************************************************/
void uart_initwBaudRate(u32 bound)
{
    /*GPIO�˿�����*/
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    /*UART1_TX   GPIOA.9*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /*�����������*/    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                     
    GPIO_Init(GPIOA, &GPIO_InitStructure);				                        
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
    
    /*UART1_RX  GPIOA.10��ʼ��*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    /*��������*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	                 
    GPIO_Init(GPIOA, &GPIO_InitStructure);				                     
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
    
    /*UART ��ʼ������*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);	                   

    /*���ڲ�����*/
    UART_InitStructure.UART_BaudRate = bound;
    /*�ֳ�Ϊ8λ���ݸ�ʽ*/    
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    /*һ��ֹͣλ*/    
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    /*����żУ��λ*/    
    UART_InitStructure.UART_Parity = UART_Parity_No;
    /*��Ӳ������������*/    
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    /*�շ�ģʽ*/
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;	                
    /*��ʼ������1*/
    UART_Init(UART1, &UART_InitStructure); 	
    /*�������ڽ����ж�*/    
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);
    /*ʹ�ܴ���1*/    
    UART_Cmd(UART1, ENABLE);                    			                    
}


/********************************************************************************************************
**������Ϣ ��void UartSendByte(u8 dat)      
**�������� ��UART��������
**������� ��dat
**������� ��
**    ��ע ��
********************************************************************************************************/
void UartSendByte(u8 dat)
{
    UART_SendData( UART1, dat);
    while(!UART_GetFlagStatus(UART1,UART_FLAG_TXEPT));
}


/********************************************************************************************************
**������Ϣ ��void UartSendGroup(u8* buf,u16 len)     
**�������� ��UART��������
**������� ��buf,len
**������� ��
**    ��ע ��
********************************************************************************************************/
void UartSendGroup(u8* buf,u16 len)
{
    while(len--)
        UartSendByte(*buf++);
}

int fputc(int ch, FILE *f)  
{  
    /* дһ���ֽڵ�USART1 */  
    UART_SendData(UART1, (uint8_t) ch);  
    /* �ȴ����ͽ��� */  
    while (UART_GetFlagStatus(UART1, UART_FLAG_TXEPT) == RESET)  
    {}  
    return ch;  
}

/********************************************************************************************************
**������Ϣ ��void LED_Init(void)     
**�������� ��LED�ܽų�ʹ�����
**������� ��
**������� ��
**    ��ע ��
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
**������Ϣ ��static void prvSetupHardware(void)     
**�������� ��Ӳ����ʹ��
**������� ��
**������� ��
**    ��ע ��Ҳ��������Ϊstatic void bspInit(void)
********************************************************************************************************/
static void prvSetupHardware( void ) 
{
    /*
     * MM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
     * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
     * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
     */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    LED_Init();
    uart_initwBaudRate(115200);
    /*1KHZ����*/
    TIM3_PWM_Init(1000-1, SystemCoreClock/1000000-1);   	
    /*����ռ�ձ�*/    
    TIM_SetCompare1(TIM3, 200);								
}
