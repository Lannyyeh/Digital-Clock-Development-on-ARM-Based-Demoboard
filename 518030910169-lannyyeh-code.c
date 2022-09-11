#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "string.h"
#include <stdio.h>
#include "pwm.c"
#include "pwm.h"
#include "hw_ints.h"


//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME				500				//500mS
#define GPIO_FLASHTIME			500				//500mS

#define b1 76446
#define b2 68106
#define b3 60676
#define b4 57274
#define b5 51020
#define b6 45454
#define b7 40496

#define c1 38167
#define c2 34014
#define c3 30303
#define c4 28653
#define c5 25510
#define c6 22727
#define c7 20243

#define none 300

void 		S800_Clock_Init(void);
void 		S800_GPIO_Init(void);
void		S800_I2C0_Init(void);
void 		Delay(uint32_t value);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void 		S800_SysTick_Init(void);
void 		S800_UART_Init(void);
void 		S800_PWM_Init(void);
void 		UARTStringPut(const char *cMessage);
void 		UARTStringPutNonBlocking(const char *cMessage);

//////////////////////////my function
struct button
{
	uint8_t number;
	uint8_t CurStatus;
	uint8_t PreStatus;
};

void set_segment(int a);
void receiving_messages(void);
int check_button(struct button *b);	
void check_carry(void);
void binding_timetable(void);
void binding_calendartable(void);
void binding_alarmtable(void);
void set_LED(void);
void set_buzzer(void);
void set_pointer(void);

//////////////////////////my function end

//const variable
uint32_t ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x40,0x00};
uint8_t studentcode[]={3,0,9,1,0,1,6,9};

//messages
char message[20];
char ret_message[20];

//software tools define
volatile uint16_t  systick_1s_counter=0,systick_40ms_counter=0,systick_500ms_counter=0;//
volatile uint8_t	 systick_1s_status=0,systick_40ms_status=0,systick_500ms_status=0; //
volatile uint8_t 	_5s_counter=0,_5s_enable=0;
volatile uint8_t uart_receive_status = 0;
volatile uint8_t cnt=0; 

volatile uint16_t	i2c_flash_cnt=0;

volatile int LED[]={0,0,0,0,0,0,0,0};

//clock and calendar variables
volatile int minute_cnt=0,second_cnt=-1,hour_cnt=0;
volatile int year_cnt=2020,month_cnt=1,date_cnt=1;
volatile int alarm1_hour_cnt=0,alarm1_minute_cnt=0,alarm2_hour_cnt=0,alarm2_minute_cnt=1;

int hour_=0,minute_=0,second_=0;
int year_=0,month_=0,date_=0;
int alarm1_hour_=0,alarm1_minute_=0,alarm2_hour_=0,alarm2_minute_=0;

volatile uint8_t time[]={0,0,10,0,0,10,0,0};
volatile uint8_t calendar[]={2,0,2,0,0,1,0,1};
volatile uint8_t alarm_set[]={0,0,0,0,0,0,0,0};

volatile uint8_t alarm1_enable=0,alarm2_enable=0;
volatile int allow_notice1=1;
volatile int is_12HourClock=0;
//with cnt:the actual count; 
//with_:to store the data from message;	
//array:to show.

volatile uint32_t first_tick=0;
volatile int blink_segment=0;
volatile int blink_1s=0;

volatile int dist_k=512;
int separate_minute[]={512,9,17,26,34,43,51,60,68,77,85,94,102,111,119,
											128,137,145,154,162,171,179,188,196,205,213,222,230,239,247,
											256,265,273,282,290,299,307,316,324,333,341,350,358,367,375,
											384,393,401,410,418,427,435,444,452,461,469,478,486,495,503};

int beepfreq1[]={none,none,
								 c1,none,c1,none,c5,none,c5,none,c6,none,c6,none,c5,c5,none,none,
								 c4,none,c4,none,c3,none,c3,none,c2,none,c2,none,c1,c1,none,none,
								 c5,none,c5,none,c4,none,c4,none,c3,none,c3,none,c2,c2,none,none,
								 c5,none,c5,none,c4,none,c4,none,c3,none,c3,none,c2,c2,none,none,
								 c1,none,c1,none,c5,none,c5,none,c6,none,c6,none,c5,c5,none,none,
								 c4,none,c4,none,c3,none,c3,none,c2,none,c2,none,c1,c1,none,none};
					
int beepfreq2[]={none,none,
							 c1,c2,c3,c1, c5,c5,c5,c3, c2,c2,c5,c5, c2,c2,c1,b6,
							 c3,c3,c3,c1, b7,b7,b7,b7, none,none,c1,b7, b6,b6,b7,b7,
							 c1,c2,b5,b5, c1,c1,c2,c3, c4,c4,c4,c3, c2,c1,c2,c2,
							 c2,c2,none,none, c1,c2,c3,c1, c5,c5,c5,c3, c2,c2,c5,c5, 
							 c2,c2,c1,b6, b6,none,b7,c1, b5,b5,b5,b5, none,b5,b6,b6,
							 b7,b7,c1,c2, b5,b5,c1,c1,  c2,c3,c4,none, c4,c3,c2,c1,
							 c1,c1,c1,c1,	c1,c1,c1,c1
							 };//
long int freqnum=0;
////////////////////////////////////////////////////////////////////////////////////////////
//buttons
struct button sw1={
	.number=0x01,
	.PreStatus=0x00,
	.CurStatus=0x00
	};
volatile int sw1_cnt=0;
struct button sw2={
	.number=0x02,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw3={
	.number=0x04,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw4={
	.number=0x08,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw5={
	.number=0x10,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw6={
	.number=0x20,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw7={
	.number=0x40,
	.PreStatus=0x00,
	.CurStatus=0x00
};
struct button sw8={
	.number=0x80,
	.PreStatus=0x00,
	.CurStatus=0x00
};
	
////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	
	for (int i=0;i<20;i++){ret_message[i]='\0';}
	message[0]='0';

	IntMasterDisable();	//

	S800_Clock_Init();
	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_SysTick_Init();
	S800_UART_Init();
	S800_PWM_Init();
	
	IntMasterEnable();	//	
	
	
	while(first_tick<1500)
		{
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[studentcode[first_tick%8]]);	//write port 1 				
			I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<(first_tick%8));				//write port 2
			SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
			first_tick++;
			}//show student code*/
	
 //main loop
	while (1)
	{		
		//1. data receving
		if(message[0]!='0')
		{ 
			int i=0;
			while(i<20){
				if(message[i]>64&&message[i]<91)message[i]=message[i]-32;
				i++; //change to a-z
			}
			receiving_messages();
			message[0]='0';
		}
		//end receiving message;
		
		//2.modify variables linked to time
		if (systick_500ms_status)
			{
				systick_500ms_status=0;
				blink_segment=1-blink_segment;									
			}
		
		if (systick_1s_status) //1s
		{
			systick_1s_status	= 0;
			second_cnt++;
			check_carry();
			blink_1s=1-blink_1s;
		
			if(_5s_enable){
				if(_5s_counter<7)_5s_counter++;
				if(_5s_counter==6){_5s_enable=0;sw1_cnt=0;}
			}
			else _5s_counter=0;
		}
		//end modifing
		
		//3. check bottons every 40ms
		if(systick_40ms_status)
		{
			systick_40ms_status=0;
			
			if(check_button(&sw1)==1){
				sw1_cnt=(sw1_cnt+1)%11;
				_5s_enable=1;
				_5s_counter=0;
				allow_notice1=0;
				freqnum=0;
				}
				switch (sw1_cnt)
				{
					case 0:
					case 1:LED[2]=LED[3]=0;LED[0]=alarm1_enable;LED[1]=alarm2_enable;break;
					case 2:
					case 3:LED[1]=LED[2]=LED[3]=0;LED[0]=blink_segment;break;
					case 4:
					case 5:LED[0]=LED[2]=LED[3]=0;LED[1]=blink_segment;break;
					case 6:
					case 7:LED[0]=LED[1]=LED[3]=0;LED[2]=blink_segment;break;	 
					case 8:
					case 9:
					case 10:LED[0]=LED[1]=LED[2]=0;LED[3]=blink_segment;break;
				}
			
			if(check_button(&sw2)==1){
					_5s_enable=1;
					_5s_counter=0;
					allow_notice1=0;
					freqnum=0;
					switch (sw1_cnt){
						case 2:alarm1_hour_cnt++;break;
						case 3:alarm1_minute_cnt++;break;
						case 4:alarm2_hour_cnt++;break;
						case 5:alarm2_minute_cnt++;break;
						case 6: hour_cnt++;break;
						case 7: minute_cnt++;break;
						case 8:	year_cnt++;break;
						case 9:	month_cnt++;break;
						case 10:date_cnt++;break;
					}
					check_carry();
			}
			if(check_button(&sw3)==1){
					_5s_enable=1;
					_5s_counter=0;
					allow_notice1=0;
					freqnum=0;
					switch (sw1_cnt){
						case 2:alarm1_hour_cnt--;break;
						case 3:alarm1_minute_cnt--;break;
						case 4:alarm2_hour_cnt--;break;
						case 5:alarm2_minute_cnt--;break;
						case 6: hour_cnt--;break;
						case 7: minute_cnt--;break;
						case 8:	year_cnt--;break;
						case 9:	month_cnt--;break;
						case 10:date_cnt--;break;
					}
					check_carry();
			}
			if(check_button(&sw4)==1){allow_notice1=0;freqnum=0;}
			if(check_button(&sw5)==1){allow_notice1=0;freqnum=0;}
			if(check_button(&sw6)==1){
				alarm1_enable=1-alarm1_enable;
				allow_notice1=0;
				freqnum=0;
			}
			if (check_button(&sw7)==1){
				alarm2_enable=1-alarm2_enable;
				allow_notice1=0;
				freqnum=0;
			}
			if (check_button(&sw8)==1){
				is_12HourClock=1-is_12HourClock;
				allow_notice1=0;
				freqnum=0;
			}
		}	
		//check bottons end
		
		//4. data management			
			binding_timetable();
			binding_calendartable();
			binding_alarmtable();
			
			set_segment(sw1_cnt);
			set_LED();
			set_buzzer();
			set_pointer();
			
			cnt = (cnt+1) % 8;
		
	}//end main loop
}

//------------- System Clock -------------------
void S800_Clock_Init(void)
{
		//ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 1000000);
}

//------------- GPIO -------------------
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));	
	
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PK5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
}

//-------------- I2C ------------------
void S800_I2C0_Init(void)
{
	uint8_t result;
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value;

	while(I2CMasterBusy(I2C0_BASE)){};	//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false); //
	I2CMasterDataPut(I2C0_BASE, RegAddr); //
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND); //
	while(I2CMasterBusBusy(I2C0_BASE));
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0; //
	Delay(100);

	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true); //
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE); //
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
		return 0; //
	Delay(100);

	return value;
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

//--------------- SysTick -----------------
void S800_SysTick_Init(void)
{
	SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); //??1ms
	SysTickEnable();
	SysTickIntEnable();
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	if (systick_1s_counter == 0) //
	{
		systick_1s_counter = 1000;
		systick_1s_status = 1;
	}
	else
		systick_1s_counter--;
	/////
	if (systick_40ms_counter == 0) //
	{
		systick_40ms_counter = 50;
		systick_40ms_status = 1;
	}
	else
		systick_40ms_counter--;
	/////
	if (systick_500ms_counter == 0) //
	{
		systick_500ms_counter = 500;
		systick_500ms_status = 1;
	}
	else
		systick_500ms_counter--;
	/////
}

//----------- UART ---------------------
void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  	GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));

	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX6_8,UART_FIFO_RX6_8);//set FIFO Level

  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
	IntEnable(INT_UART0);
	
	UARTStringPut("\r\nHello, world!\r\n");
}

void 	S800_PWM_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); 
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { } 
	
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); 
	
	PWMGenConfigure(PWM0_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN |PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,300); 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,300/6); 
	
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true); 
	PWMGenEnable(PWM0_BASE,PWM_GEN_3); 
}

void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}

void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		if (UARTSpaceAvail(UART0_BASE))
			UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}

/*
	Corresponding to the startup_TM4C129.s vector table UART0_Handler interrupt program name
*/
void UART0_Handler(void)
{
	int32_t uart0_int_status;
	int k=-1;
  char uart_receive_char;
	
  uart0_int_status = UARTIntStatus(UART0_BASE, true);			// Get the interrrupt status.
  UARTIntClear(UART0_BASE, uart0_int_status);							//Clear the asserted interrupts

	if (uart0_int_status & (UART_INT_RX | UART_INT_RT)) 	
	{
		
			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1 );	//turn on PN1

			while(UARTCharsAvail(UART0_BASE))    										// Loop while there are characters in the receive FIFO.
			{
				uart_receive_char = UARTCharGetNonBlocking(UART0_BASE);
				SysCtlDelay(ui32SysClock/30000);//0.1ms
				message[++k]=uart_receive_char;
			}
			GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,0 );		//turn off PN1
			
	}
}
void receiving_messages(void)
{
			if(strncmp(message,"tset",4)==0){
				sscanf(message+4,"%2d",&hour_);
				sscanf(message+7,"%2d",&minute_);
				sscanf(message+10,"%2d",&second_);
				
				second_cnt=second_;
				minute_cnt=minute_;
				hour_cnt=hour_;
				
				check_carry();
				binding_timetable();
			}
			
			if(strncmp(message,"dset",4)==0){
				sscanf(message+4,"%4d",&year_);
				sscanf(message+9,"%2d",&month_);
				sscanf(message+12,"%2d",&date_);
				
				year_cnt=year_;
				month_cnt=month_;
				date_cnt=date_;
				
				check_carry();
				binding_calendartable();
			}
			
			if(strncmp(message,"setalm1",7)==0)
				{
					sscanf(message+7,"%2d",&alarm1_hour_);
					sscanf(message+10,"%2d",&alarm1_minute_);
					alarm1_hour_cnt=alarm1_hour_;
					alarm1_minute_cnt=alarm1_minute_;
					binding_alarmtable();
				}
				
			if(strncmp(message,"setalm2",7)==0)
				{
					sscanf(message+7,"%2d",&alarm2_hour_);
					sscanf(message+10,"%2d",&alarm2_minute_);
					alarm2_hour_cnt=alarm2_hour_;
					alarm2_minute_cnt=alarm2_minute_;
					binding_alarmtable();
				}
				
			if(strncmp(message,"getdate",7)==0){
				ret_message[0]='D';ret_message[1]='a';ret_message[2]='t';ret_message[3]='e';ret_message[4]=':';
				ret_message[5]=calendar[0]+48;
				ret_message[6]=calendar[1]+48;
				ret_message[7]=calendar[2]+48;
				ret_message[8]=calendar[3]+48;
				ret_message[9]='-';
				ret_message[10]=calendar[4]+48;
				ret_message[11]=calendar[5]+48;
				ret_message[12]='-';
				ret_message[13]=calendar[6]+48;
				ret_message[14]=calendar[7]+48;
				ret_message[15]='\0';
				UARTStringPutNonBlocking(ret_message);
			}
			
			if(strncmp(message,"gettime",7)==0){
				ret_message[0]='T';ret_message[1]='i';ret_message[2]='m';ret_message[3]='e';ret_message[4]=':';
				ret_message[5]=time[0]+48;
				ret_message[6]=time[1]+48;
				ret_message[7]='-';
				ret_message[8]=time[3]+48;
				ret_message[9]=time[4]+48;
				ret_message[10]='-';
				ret_message[11]=time[6]+48;
				ret_message[12]=time[7]+48;
				ret_message[13]='\0';
				UARTStringPutNonBlocking(ret_message);
			}
			if(strncmp(message,"reset",7)==0){SysCtlReset();}
	}

void set_segment(int a){
	switch(a)
	{	
			//show clock
		case 0:	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[time[cnt]]);	//write port 1 				
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
		
		//show date
		case 1: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[calendar[cnt]]);	//write port 1 				
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
		
		//show alarm1 setting,blink hour bit and dim alarm2
		case 2: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[alarm_set[cnt]]);	//write port 1 				
						
						if(cnt>3)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0<<cnt);
						else if(cnt==0|cnt==1)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
		
		//show alarm1 setting,blink the minute bit and dim alarm2
		case 3:	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[alarm_set[cnt]]);	//write port 1 		
						if(cnt>3)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0<<cnt);
						else if(cnt==2|cnt==3)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
		
		//show alarm2 setting,blink the hour bit and dim alarm1			
		case 4:I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[alarm_set[cnt]]);	//write port 1 				
						if (cnt<4)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0<<cnt);
						else if(cnt==4|cnt==5)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//show alarm2 setting,blink the minute bit and dim alarm1				
		case 5:I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[alarm_set[cnt]]);	//write port 1 				
						if (cnt<4)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT2,0<<cnt);							
						else if(cnt==6|cnt==7)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);				//write port 2
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//setting clock-hour,blink hour bit 			
		case 6: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[time[cnt]]);	//write port 1 				
						if(cnt==0|cnt==1)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);				//write port 2
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//setting clock-minute,blink minute bit				
		case 7: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[time[cnt]]);	//write port 1 				
						if(cnt==3|cnt==4)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);				//write port 2
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//setting calendar-year,blink				
		case 8: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[calendar[cnt]]);	//write port 1 				
						if(cnt==0|cnt==1|cnt==2|cnt==3)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);				//write port 2
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//setting calendar-month,blink				
		case 9: I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[calendar[cnt]]);	//write port 1 				
						if(cnt==4|cnt==5)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);				//write port 2
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
						
		//setting calendar-date,blink				
		case 10:I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x00);
						I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[calendar[cnt]]);	//write port 1 				
						if(cnt==6|cnt==7)
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,blink_segment<<cnt);				//write port 2
						else 
							I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1<<cnt);
						SysCtlDelay(ui32SysClock/4500);//  (1/1500)s
						break;
	}
}

int check_button(struct button *b)
{
	(*b).PreStatus=(*b).CurStatus;
	(*b).CurStatus=~I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
	if( (*b).PreStatus==0&&(*b).CurStatus==(*b).number)return 1;
	else return 0;
}

void check_carry(void)
{
		if (second_cnt>=60){minute_cnt+=(second_cnt)/60;second_cnt=second_cnt%60;allow_notice1=1;}			
		if (minute_cnt>=60){hour_cnt+=(minute_cnt)/60;minute_cnt=minute_cnt%60;}  
		if (hour_cnt>=24){date_cnt+=(hour_cnt/24),hour_cnt=hour_cnt%24;}
		if (date_cnt>=31){month_cnt+=1,date_cnt=1;}
		if (month_cnt>=13){year_cnt+=1,month_cnt=1;}
		
		if (alarm1_minute_cnt>=60){alarm1_hour_cnt++;alarm1_minute_cnt%=60;}
		if (alarm1_hour_cnt>=24){alarm1_hour_cnt%=24;}
		if (alarm2_minute_cnt>=60){alarm2_hour_cnt++;alarm2_minute_cnt%=60;}
		if (alarm2_hour_cnt>=24){alarm2_hour_cnt%=24;}
		
		if (second_cnt<0){second_cnt=59;}			
		if (minute_cnt<0){minute_cnt=59;}  
		if (hour_cnt<0){hour_cnt=23;}
		if (date_cnt<1){date_cnt=30;}
		if (month_cnt<1){month_cnt=12;}
		
		if (alarm1_minute_cnt<0){alarm1_minute_cnt=59;}
		if (alarm1_hour_cnt<0){alarm1_hour_cnt=23;}
		if (alarm2_minute_cnt<0){alarm2_minute_cnt=59;}
		if (alarm2_hour_cnt<0){alarm2_hour_cnt=23;}
}

void binding_timetable(void)
{
		if(is_12HourClock){time[0]=(hour_cnt%12)/10;time[1]=(hour_cnt%12)%10;}
		else {time[0]=hour_cnt/10;time[1]=hour_cnt%10;}
		time[3]=minute_cnt/10;time[4]=minute_cnt%10;
		time[6]=second_cnt/10;time[7]=second_cnt%10;
}
void binding_calendartable(void)
{
		calendar[0]=year_cnt/1000;calendar[1]=(year_cnt/100)%10;
		calendar[2]=(year_cnt/10)%10;calendar[3]=year_cnt%10;
		calendar[4]=month_cnt/10;calendar[5]=month_cnt%10;
		calendar[6]=date_cnt/10;calendar[7]=date_cnt%10;
}
void binding_alarmtable(void)
{
	alarm_set[0]=alarm1_hour_cnt/10;
	alarm_set[1]=alarm1_hour_cnt%10;
	alarm_set[2]=alarm1_minute_cnt/10;
	alarm_set[3]=alarm1_minute_cnt%10;
	alarm_set[4]=alarm2_hour_cnt/10;
	alarm_set[5]=alarm2_hour_cnt%10;
	alarm_set[6]=alarm2_minute_cnt/10;
	alarm_set[7]=alarm2_minute_cnt%10;
}
void set_LED(void)
{
		LED[4]=((is_12HourClock)&&hour_cnt>=12)?1:0;
		if((hour_cnt==alarm1_hour_cnt)&&(minute_cnt==alarm1_minute_cnt)&&alarm1_enable&&(allow_notice1==1))
			{
				LED[0]=blink_segment;
			}
		if((hour_cnt==alarm2_hour_cnt)&&(minute_cnt==alarm2_minute_cnt)&&alarm2_enable&&(allow_notice1==1))
			{
				LED[1]=blink_1s;
			}	
		
		int k=LED[0]+LED[1]*2+LED[2]*4+LED[3]*8+LED[4]*16+LED[5]*32+LED[6]*64+LED[7]*128;
		I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~(k));
}
void set_buzzer(void)
{
	if((hour_cnt==alarm1_hour_cnt)&&(minute_cnt==alarm1_minute_cnt)&&alarm1_enable&&(allow_notice1==1)){
		freqnum=(freqnum+1)%29400;
		if(freqnum%300==0){
			PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,beepfreq1[(freqnum/300)%98]);
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,beepfreq1[(freqnum/300)%98]/6); 
			}
	}
	else if((hour_cnt==alarm2_hour_cnt)&&(minute_cnt==alarm2_minute_cnt)&&alarm2_enable&&(allow_notice1==1)){
			freqnum=(freqnum+1)%26000;
			if(freqnum%250==0){
			PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,beepfreq2[(freqnum/250)%104]);
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,beepfreq2[(freqnum/250)%104]/6); 
			}
		}
	else 
		{
			PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,300);
			PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,300/6); }
}
void set_pointer(void)
{
	if(dist_k!=separate_minute[minute_cnt]){
		if(((dist_k<separate_minute[minute_cnt])&&separate_minute[minute_cnt]-dist_k<256)||
			 ((dist_k>separate_minute[minute_cnt])&&dist_k-separate_minute[minute_cnt]>256)){
				dist_k=(dist_k+1)%513;
		
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);							// Turn off the LED.
								
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);							// Turn off the LED.
								
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);							// Turn off the LED.
									
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);							// Turn off the LED.
			}

		else if(((dist_k<separate_minute[minute_cnt])&&separate_minute[minute_cnt]-dist_k>256)||
				   ((dist_k>separate_minute[minute_cnt])&&dist_k-separate_minute[minute_cnt]<256))
				{
				if(dist_k==0)dist_k=512;
				dist_k=(dist_k-1)%513;
		
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);							// Turn off the LED.
								
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);							// Turn off the LED.
								
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);							// Turn off the LED.
									
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);			// Turn on the LED.
				SysCtlDelay(3*ui32SysClock/4500); //
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);							// Turn off the LED.
			}
	}
}
