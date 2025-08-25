#include <stdint.h>
#include <stdio.h>
#include "tm4c1294ncpdt.h"
#include "Systick.h"
#include "PLL.h"

//stepper motor phases 
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTL_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTL_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTM_PCTL_R = (GPIO_PORTM_PCTL_R&0xFFFFFF0F)+0x00000000;
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

//keypad inputs
void PortM0M1M2M3_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; //allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00000000; // Make PM0:PM3 inputs, reading if the button is pressed or not
	GPIO_PORTM_DEN_R = 0b00001111; // Enable PM0:PM3
	
	GPIO_PORTM_PUR_R |= 0x0F; //enabling pull up resistors (now all PM inputs are set to high automatically until button is pressed)
return;
}

//keypad outputs
void PortE0E1E2E3_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // allow time for clock to stabilize
	//GPIO_PORTE_DIR_R = 0b00001111;
	GPIO_PORTE_DEN_R = 0b00001111; // Enable PE0:PE3 
return;
}

//Turns on D2, D1
void PortN0N1_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
GPIO_PORTN_DIR_R=0b00000011; //Make PN0 and PN1 outputs, to turn on LED's
GPIO_PORTN_DEN_R=0b00000011; //Enable PN0 and PN1
return;
}

//Turns on D3, D4
void PortF0F4_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; //activate the clock for Port F
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};//allow time for clock to stabilize
GPIO_PORTF_DIR_R=0b00010001; //Make PF0 and PF4 outputs, to turn on LED's
GPIO_PORTF_DEN_R=0b00010001;
return;
}

//Rear motors 
void RearMotor_Reverse(void) {
    GPIO_PORTH_DATA_R = 0b00000101; // IN1=1, IN2=0, IN3=1, IN4=0
}

void RearMotor_Forward(void) {
    GPIO_PORTH_DATA_R = 0b00001010; // IN1=0, IN2=1, IN3=0, IN4=1
}

void RearMotor_Stop(void) {
    GPIO_PORTH_DATA_R = 0b00000000; // IN1=0, IN2=0, IN3=0, IN4=0
}

//Front motors
void FrontMotor_Reverse(void) {
    GPIO_PORTL_DATA_R = 0b00000101; // IN1=1, IN2=0, IN3=1, IN4=0
}

void FrontMotor_Forward(void) {
    GPIO_PORTL_DATA_R = 0b00001010; // IN1=0, IN2=1, IN3=0, IN4=1
}

void FrontMotor_Stop(void) {
    GPIO_PORTL_DATA_R = 0b00000000; //IN1=0, IN2=0, IN3=0, IN4=0
}


int main(void){
	PLL_Init();
	SysTick_Init();
	PortE0E1E2E3_Init();
	PortM0M1M2M3_Init();
	PortL_Init();
	PortN0N1_Init();
	PortF0F4_Init();
	PortH_Init();
	
	while(1){
		RearMotor_Forward();
		FrontMotor_Forward();
    SysTick_Wait10ms(300);  // run 2 sec
		GPIO_PORTN_DATA_R = 0x01; 
		
		RearMotor_Stop();
		FrontMotor_Stop();
		SysTick_Wait10ms(300);  // stop 1 sec
		
		RearMotor_Reverse();
		FrontMotor_Reverse();
		SysTick_Wait10ms(300); 
		GPIO_PORTN_DATA_R = 0b00000010;
    
		RearMotor_Stop();
		FrontMotor_Stop();
		SysTick_Wait10ms(300);  // stop 1 sec
	}
}

