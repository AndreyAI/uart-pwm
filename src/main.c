#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stdlib.h"
#include "stdio.h"



#define SYSCLK 16000000
#define PRESCALER 15

GPIO_InitTypeDef port;
TIM_TimeBaseInitTypeDef timer;
TIM_OCInitTypeDef timerPWM;
USART_InitTypeDef UART_Init;
uint8_t rxdata;
uint8_t TransmitData[80];
uint8_t receivedData[10];
uint8_t receivedDataCounter = 0;
uint8_t transmitDataCounter = 0;
uint16_t him_x, him_y;
char *a, *p;
double per, z; // для double2string



void SerialPutChar(uint8_t c)
{
    USART_SendData(USART1, c);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
}

void Serial_PutString(uint8_t *s)
{
    while (*s != '\0')
    {
        SerialPutChar(*s);
        s++;
    }
}

void Write_Float(double f)
{
	int cel = f;
	double drob = f - cel;
	int drobcel = drob * 1000;
	p = itoa(cel,TransmitData,10);
	Serial_PutString(TransmitData);
	Serial_PutString(",");
	p = itoa(drobcel,TransmitData,10);
	Serial_PutString(TransmitData);
	Serial_PutString("%");

}

void Obnovlenie(void)
{
	Serial_PutString("\r\nkoordinaty\r\n\0");
			Serial_PutString("x = ");
			p = itoa(him_x-500,TransmitData,10);
			Serial_PutString(TransmitData);
			Serial_PutString(" (iz 1800)");
			Serial_PutString(" y = ");
			p = itoa(him_y-1250,TransmitData,10);
			Serial_PutString(TransmitData);
			Serial_PutString(" (iz 700)");
			Serial_PutString("\r\nx = ");
				z = (him_x-500.0)/18.0;
				Write_Float(z);
				Serial_PutString(" y = ");
				z = (him_y-1250.0)/7.0;

				Write_Float(z);

}


 void UART_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);// тактирование порта

	port.GPIO_Mode = GPIO_Mode_AF;
	port.GPIO_Pin = GPIO_Pin_9;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	port.GPIO_OType = GPIO_OType_PP;
	port.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOA, &port);

	 port.GPIO_Mode = GPIO_Mode_AF;
	 port.GPIO_Pin = GPIO_Pin_10;
	 port.GPIO_Speed = GPIO_Speed_50MHz;
	 port.GPIO_OType = GPIO_OType_PP;
	 port.GPIO_PuPd = GPIO_PuPd_UP;

	 GPIO_Init(GPIOA, &port);

	 //назначаем альтернативные функции
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //PA9 to TX USART1
	 GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //PA10 to RX USART1

	//тактирование UART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//настраиваем уарт заполняя структуру
	UART_Init.USART_BaudRate = 9600;// скорость
	UART_Init.USART_WordLength = USART_WordLength_8b; //8 бит данных
	UART_Init.USART_StopBits = USART_StopBits_1; //один стоп бит
	UART_Init.USART_Parity = USART_Parity_No; //четность - нет
	UART_Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // управлени потоком - нет
	UART_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       // разрешаем прием и передачу

	USART_Init(USART1, &UART_Init);

	NVIC_EnableIRQ(USART1_IRQn);

	USART_Cmd(USART1, ENABLE);
}

void servo_init(void) {
	//ПЕРВЫЙ ШИМ
	//Отсюда будем снимать сигнал настраиваем порт

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); // hanges the mapping of the specified pin. без нее никак

    port.GPIO_Pin = GPIO_Pin_6;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_OType   = GPIO_OType_PP;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &port);

    //TIM_TimeBaseStructInit(&timer); //Конфигурируем таймер: чтобы получить частоту PWM 50 Гц

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    timer.TIM_Prescaler = PRESCALER;
    timer.TIM_Period = 19999 ; // частота ШИМ 50Гц
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);

    //TIM_OCStructInit(&timerPWM);

    timerPWM.TIM_Pulse = 1800;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &timerPWM);


    TIM_Cmd(TIM4, ENABLE);

    //ВТОРОЙ ШИМ


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); // hanges the mapping of the specified pin. без нее никак

    port.GPIO_Pin = GPIO_Pin_0;
    port.GPIO_Mode = GPIO_Mode_AF;
    port.GPIO_OType   = GPIO_OType_PP;

    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);

        //TIM_TimeBaseStructInit(&timer); //Конфигурируем таймер: чтобы получить частоту PWM 50 кГц

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    timer.TIM_Prescaler = PRESCALER;
    timer.TIM_Period = 19999 ; // частота ШИМ 50КГц
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &timer);

        //TIM_OCStructInit(&timerPWM);

    timerPWM.TIM_Pulse = 1630;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM5, &timerPWM);


    TIM_Cmd(TIM5, ENABLE);

}

int main(void)
{
	__enable_irq();

	UART_init();

	servo_init();

	him_x = 1800;
	him_y = 1630;

	// Включаем прерывание по окончанию передачи
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	    while(1)

	    {
	    }
}


void USART1_IRQHandler()
{
// Убеждаемся, что прерывание вызвано новыми данными в регистре данных
if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
{
	rxdata = USART_ReceiveData(USART1);
	receivedData[receivedDataCounter] = rxdata;
	receivedDataCounter++;
}

//Увеличиваем хШИМ на 1
if (rxdata == 'w'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
him_x++;
TIM4->CCR1 = him_x;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

//Уменьшаем хШИМ на1
if (rxdata == 's'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
him_x--;
TIM4->CCR1 = him_x;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

//уШИМ увеличиваем на1
if (rxdata == 'q'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
him_y++;
TIM5->CCR1 = him_y;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

//Уменьшаем уШИМ на1
if (rxdata == 'a'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
him_y--;
TIM5->CCR1 = him_y;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}



if (rxdata == 'x'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
a = receivedData;
him_x = atoi (a)+500;
TIM4->CCR1 = him_x;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);//}*/

}


if (rxdata == 'y'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
a = receivedData;
him_y = atoi (a)+1250;
TIM5->CCR1 = him_y;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

if (rxdata == 'p'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
a = receivedData;
per = (18*atof (a))+500;
him_x = per;
TIM4->CCR1 = him_x;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

if (rxdata == 'l'){
USART_ClearITPendingBit(USART1, USART_IT_RXNE);
receivedDataCounter = 0;
a = receivedData;
per = (7*atof (a))+1250;
him_y = per;
TIM5->CCR1 = him_y;

		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

if (rxdata == 'c'){
	him_x = 1800;
		him_y = 1630;
		TIM4->CCR1 = him_x;
		TIM5->CCR1 = him_y;



		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
}

if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {



		Obnovlenie();

		USART_ClearITPendingBit(USART1, USART_IT_TC);

		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    }

}





		//Serial_PutString("Tekushee znachenie");
		//USART_SendData(USART1, tekst[transmitDataCounter] );
        //transmitDataCounter++;
        //if (tekst[transmitDataCounter] == '*'){
        //USART_SendData(USART1, him);








/* if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_SendData(USART1, usartData[usartCounter]);
        usartCounter++;
    }
    USART_ClearITPendingBit(USART1, USART_IT_TC);
}
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * void led_init(void){

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


        // Настраиваем порты
      // создаем структуру

        ledinit.GPIO_Mode = GPIO_Mode_OUT;  // направление - выход
        ledinit.GPIO_OType = GPIO_OType_PP;  // Двухтактный выход
        ledinit.GPIO_PuPd = GPIO_PuPd_NOPULL;  // Без подтяжки
        ledinit.GPIO_Speed = GPIO_Speed_2MHz;  // Скорость низкая
        ledinit.GPIO_Pin = GPIO_Pin_9;

        GPIO_Init(GPIOC, &ledinit);

}
 *
 *
 *
 *
 *
 *
 * #include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

// Функция программной задержки
void Soft_Delay(volatile uint32_t number)
{
        while(number--);
}

int main(void)
{

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


        // Настраиваем порты
        GPIO_InitTypeDef  ledinit;  // создаем структуру

        ledinit.GPIO_Mode = GPIO_Mode_OUT;  // направление - выход
        ledinit.GPIO_OType = GPIO_OType_PP;  // Двухтактный выход
        ledinit.GPIO_PuPd = GPIO_PuPd_NOPULL;  // Без подтяжки
        ledinit.GPIO_Speed = GPIO_Speed_2MHz;  // Скорость низкая
        ledinit.GPIO_Pin = GPIO_Pin_9;

        GPIO_Init(GPIOC, &ledinit);



        TIM_TimeBaseInitTypeDef timeinit;

        t

    while(1)
    {
        // Включаем светодиод
    	GPIO_SetBits(GPIOC, GPIO_Pin_9);

        // Делаем паузу
        Soft_Delay(0x000FFFFF);

        // Теперь выключаем светодиод
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);

        // Делаем паузу
        Soft_Delay(0x000FFFFF);
    }
} */

/*if((receivedData[0] & receivedData[1]) == '1' ){

	    GPIO_SetBits(GPIOC, GPIO_Pin_9);

	    // Делаем паузу
	    Soft_Delay(0x000FFFFF);

	    // Теперь выключаем светодиод
	    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	    // Делаем паузу
	    Soft_Delay(0x000FFFFF);}

	    } */

	    // __NOP();
	    // }



