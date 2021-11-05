#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "SysTick.h"
#include "onboardLEDs.h"

#define DELAY 1

//Flash D1
void FlashLED1(int count) {
	while(count--) {
		GPIO_PORTN_DATA_R ^= 0b00000010; 								//hello world!
		SysTick_Wait10ms(DELAY);														//.05s delay
		GPIO_PORTN_DATA_R ^= 0b00000010;			
		SysTick_Wait10ms(DELAY);														//.05s delay
	}
}

//Flash D2
void FlashLED2(int count) {
	while(count--) {
		GPIO_PORTN_DATA_R ^= 0b00000001; 								//hello world!
		SysTick_Wait10ms(DELAY);														//.05s delay
		GPIO_PORTN_DATA_R ^= 0b00000001;			
		SysTick_Wait10ms(DELAY);														//.05s delay			
	}
}

//Flash D3
void FlashLED3(int count) {
	while(count--) {
		GPIO_PORTF_DATA_R ^= 0b00010000; 								//hello world!
		SysTick_Wait10ms(DELAY);														//.05s delay
		GPIO_PORTF_DATA_R ^= 0b00010000;			
		SysTick_Wait10ms(DELAY);														//.05s delay			
	}
}

//Flash D4
void FlashLED4(int count) {
	while(count--) {
		GPIO_PORTF_DATA_R ^= 0b00000001; 								//hello world!
		SysTick_Wait10ms(DELAY);														//.05s delay
		GPIO_PORTF_DATA_R ^= 0b00000001;			
		SysTick_Wait10ms(DELAY);														//.05s delay			
	}
}

void FlashAllLEDs(){
	GPIO_PORTN_DATA_R ^= 0b00000011; 								//hello world!
	GPIO_PORTF_DATA_R ^= 0b00010001; 								//hello world!	
	SysTick_Wait10ms(25);																//.25s delay
	GPIO_PORTN_DATA_R ^= 0b00000011;			
	GPIO_PORTF_DATA_R ^= 0b00010001; 								//goodbye world!	
	SysTick_Wait10ms(25);																//.25s delay			
}

void loading() {
	uint8_t delay = 10;
	GPIO_PORTN_DATA_R ^= 0x2;//turn on D1
	SysTick_Wait10ms(delay);
	GPIO_PORTN_DATA_R ^= 0x3;//turn off D1 and on D2
	SysTick_Wait10ms(delay);
	GPIO_PORTN_DATA_R ^= 0x1;//turn off D2
	GPIO_PORTF_DATA_R ^= 0x10;//turn on D3
	SysTick_Wait10ms(delay);
	GPIO_PORTF_DATA_R ^= 0x11;//turn off D3 and on D4
	SysTick_Wait10ms(delay);
	GPIO_PORTF_DATA_R ^= 0x1;//turn oof D4
}

void FlashI2CTx() {
	FlashLED3(1);
}

void FlashI2CRx() {
	FlashLED4(1);
}

//Flash Error D1&D2&D3&D4
void FlashI2CError(int count) {
	while(count--) {
		FlashAllLEDs();
	}
}

// Initialize onboard LEDs
void onboardLEDs_Init(void){
	//Use PortN onboard LEDs	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;						// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x03;        										// make PN0 out (PN0 built-in LED1)
  GPIO_PORTN_AFSEL_R &= ~0x03;     										// disable alt funct on PN0,PN1
  GPIO_PORTN_DEN_R |= 0x03;        										// enable digital I/O on PN0,PN1
  GPIO_PORTN_AMSEL_R &= ~0x03;     										// disable analog functionality on PN0,PN1
	
	//Use PortF onboard LEDs	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;						// activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};		// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x11;        										// make PN4,0 out 
  GPIO_PORTF_AFSEL_R &= ~0x11;     										// disable alt funct on PF4,PF0
  GPIO_PORTF_DEN_R |= 0x11;        										// enable digital I/O on PF4,PF0
  GPIO_PORTF_AMSEL_R &= ~0x011;     									// disable analog functionality on PF4,PF0
	
	FlashAllLEDs();
	return;
}
