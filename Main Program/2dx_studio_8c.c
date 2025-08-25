/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

//I2C Definitions 
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTL_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTL_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
 
  GPIO_PORTL_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

void PortH_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								// make all pins outputs 
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								// disable alt funct 
  GPIO_PORTH_DEN_R |= 0xFF;        								// enable digital I/O on all pins 
																							
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality 	
	return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

//Pulse Width Modulation 
void PWM_Init(void){
	SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0; 					//activate clock for PWM 
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				//activate clock for Port F (using F1 for M0PWM1 and F2 for M0PWM2)
	GPIO_PORTF_AFSEL_R |= 0x06; 										//activating peripherals on PF1 and PF2 (0b00001100)
	GPIO_PORTF_PCTL_R |= (0x06 << 4) | (0x06 << 8);		//setting bit fields 4-7 and 8-11 to 6 (see the PWM configuration doc for more details) 
	//PWM0 --> CC = (1 << 0) | (0x0 << 1); 
	PWM0_CC_R = (1 << 0) | (0x0 << 1);							//USEPWMDIV = 1, PWMDIV = 0 (divide by 2)
		
	//PWM1 Generator 0
	PWM0_0_CTL_R = 0x0; 														//resetting counter
	PWM0_0_GENB_R = 0x08C; 													//
	PWM0_0_LOAD_R = 0x018F;
	//PWM0_0_CMPB_R = 0x012B; 															
	PWM0_0_CTL_R = 0x01; 														//starting counter for generator 0 
	
	//PWM2 Generator 1 
	PWM0_1_CTL_R = 0x0;															//resetting counter
	PWM0_1_GENA_R = 0x80C; 
	PWM0_1_LOAD_R = 0x018F;
	//PWM0_1_CMPA_R = 0x0063; 
	PWM0_1_CTL_R = 0x01;														//starting counter for generator 1 
		
	PWM0_CTL_R = 0x01; 															//providing master control over PWM generation blocks 
	PWM0_ENABLE_R = 0x06;														//enabling the PWM1 and PWM2
}


//Rear motors 
void RearMotor_Reverse(void) {
    GPIO_PORTH_DATA_R = 0b00000101; // IN1=1, IN2=0, IN3=1, IN4=0
}

void RearMotor_Forward(void) {
    GPIO_PORTH_DATA_R = 0b00001010; // IN1=0, IN2=1, IN3=0, IN4=1
}

//Front motors
void FrontMotor_Reverse(void) {
    GPIO_PORTL_DATA_R = 0b00000101; // IN1=1, IN2=0, IN3=1, IN4=0
}

void FrontMotor_Forward(void) {
    GPIO_PORTL_DATA_R = 0b00001010; // IN1=0, IN2=1, IN3=0, IN4=1
}

//braking
void Motor_Stop(void) {
    GPIO_PORTL_DATA_R = 0b00000000; //IN1=0, IN2=0, IN3=0, IN4=0
		GPIO_PORTH_DATA_R = 0b00000000;
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  //uint16_t SignalRate;
  //uint16_t AmbientRate;
  //uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	int delay = 1; 
	int Stop_Distance = 100; 
	
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PWM_Init();
	PortH_Init();
	PortL_Init();

	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	
	//reading distances 
	//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor
		
		while(1){	
		//running the motor
			
			//car is driving
			FrontMotor_Forward();
			RearMotor_Forward(); 
			
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			FlashLED4(1);
			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			sprintf(printf_buffer,"%u, %u\r\n", RangeStatus, Distance);
			UART_printf(printf_buffer);
			if (Distance < Stop_Distance) break;
	
		}
	
	//slowing motor down 
	UART_printf("Stop Distance exceeded. Slowing down...\r\n");
	
	//write out PWM code here
	for (int duty = 0; duty >= 0; duty -= 5){
		int cmp_value = (399*(100 - duty))/100; 
		PWM0_0_CMPB_R = cmp_value; 
		PWM0_1_CMPA_R = cmp_value; 
		
		SysTick_Wait10ms(50);
	}
		
	Motor_Stop();

	UART_printf("Stopped\r\n");
	VL53L1X_StopRanging(dev);

}




