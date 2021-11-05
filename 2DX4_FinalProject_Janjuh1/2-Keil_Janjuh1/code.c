//2DX4 Project - Haroon Janjua
//400196693
//janjuh1
//Bus Speed: 48(MHz), change in PLL.h where I change PSYSDIV to 9
//Distance Status: PN1(LED 1)
//Displacement Stutus: PL5

#include <stdint.h>
#include <math.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "Systick.h"
#include "PLL.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "I2C0.h"

#define isInterrupt 1	//the value that indicates if program is in interrupt mode/pulling mode (1 = interrupt mode)
#define PI 3.14159265

//Initializing Port L0-L3 to output to stepper motor and L5 for external LED representing displacement status
void PortL_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};
	GPIO_PORTL_DIR_R |= 0b101111;
	GPIO_PORTL_DEN_R |= 0b101111;
}

//Initializing Port G0 so that ToF can be reset using XSHUT
void PortG_Init(void) {
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};
	GPIO_PORTG_DIR_R &= ~0b1;			//Set PG0 to input
	GPIO_PORTG_AFSEL_R &= ~0b1;		//Disable Alt funct on PG0
	GPIO_PORTG_DEN_R |= 0b1;			//Enable Digital I/O on PG0
	GPIO_PORTG_AMSEL_R &= ~0b1;		//Disable Analog Funct on PG0
}

int				slicePoints = 8;	//the amount of points per 1 slice
int				arr[40] = {-1};		//Array which consists of the ToF measurement values
int				zOffset = 0;			//how high is the device, in my case 0
uint16_t	dev = 0x52;				//Default address for ToF sensor
uint8_t		status = 0;				//Var which is the current ToF status bit
uint8_t		ToFSensor = 1;		//0=Left, 1=Center(default), 2=Right
uint16_t	Distance;					//tooken care of in python, here you can go on forever

//Function which sets up the ToF sensor
void ToF_Init(void) {
	uint8_t		sensorState = 0; //stores ToF state if 0 ToF is off

	//Starting up the ToF chip
	while(sensorState==0) {
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
	}
	
	//Flashing all LEDS to indicate this startup
	FlashAllLEDs();

	//Clearing interrupt used for the next interrupt
	status = VL53L1X_ClearInterrupt(dev);

	//Initializing ToF sensor with default values
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	//Enabling the ranging of the ToF
	status = VL53L1X_StartRanging(dev);
	Status_Check("StartRanging", status);
	return;
}

//Function which gets a distance measurement from the ToF chip and returns it
uint8_t getTOFDistance(void) {
	uint8_t		dataReady = 0;
	
	//Waiting for ToF to be ready for sensing
	while (dataReady == 0) {
		status = VL53L1X_CheckForDataReady(dev, &dataReady);
		VL53L1_WaitMs(dev, 5);
	}
	status = VL53L1X_GetDistance(dev, &Distance); //Getting the distance var
	FlashLED2(1); //Flashing LED 2 (PN0) on the onboard MSPY board to indicate successful reading

	//Reset the interrupt for the ToF sensor for next interrupt
	status = VL53L1X_ClearInterrupt(dev); 
	return Distance; //Returning the distance
}

//Func that applies XSHUT to provide an active-low shutdown
//Puts ToF into hardware standby (input not level shifted)
//THIS IS NOT USED IN THIS CODE
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01; //Set PG0 to output
    GPIO_PORTG_DATA_R &= 0b11111110; //Set PGO to a low reading
    FlashAllLEDs(); //Flash All LEDS
    SysTick_Wait10ms(10); //Wait
    GPIO_PORTG_DIR_R &= ~0x01; //Set PG0 to Input (HIZ)
}

//spins motor in clockwise direction
void cclockwise(int speed) {
	SysTick_WaitHalfms(speed);
	GPIO_PORTL_DATA_R = 0b1001;
	SysTick_WaitHalfms(speed);
	GPIO_PORTL_DATA_R = 0b0011;
	SysTick_WaitHalfms(speed);
	GPIO_PORTL_DATA_R = 0b0110;
	SysTick_WaitHalfms(speed);
	GPIO_PORTL_DATA_R = 0b1100;
}

//Enables interrupts
void EnableInt(void) {
	__asm("    cpsie   i\n");
}

//Disables interrupts
void DisableInt(void) {
	__asm("    cpsid   i\n");
}

//Waits for interrupt
void WaitForInt(void) {
	__asm("    wfi\n");
}

//GPIO Port J = Vector 67
//Bit in interrupt register = 51
//Function that sets onboard button PJ1 as an interrupt
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					//activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	//allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    		//Set PJ1 as input

  GPIO_PORTJ_DEN_R |= 0x03;     		//Digital Enable PJ0&PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000FF; //Set PJ0&PJ1 as GPIO
	GPIO_PORTJ_AMSEL_R &= ~0x03;			//Disable analog funct for PJ0&PJ1
	GPIO_PORTJ_PUR_R |= 0x03;					//Enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x03;     		//Set PJ0&PJ1 as edge-sensitive
  GPIO_PORTJ_IBE_R &= ~0x03;    		//Set PJ0&PJ1 as not both edges sensitive
  GPIO_PORTJ_IEV_R &= ~0x03;    		//Set PJ0&PJ1 as falling edge sensitive event
  GPIO_PORTJ_ICR_R = 0x03;      		//Clear flag1
  GPIO_PORTJ_IM_R |= 0x03;					//Set Arm Interrupt for PJ0&PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; //Set priority to 5
  NVIC_EN1_R |= 0x00080000;					//Enable Interrupt 67 in NVIC
  EnableInt();
}

//Current state of the program 0-wait/send data if possible, 1-get data
int state = 0;

//The Interrupt Handler when PJ1 or PJ2 button is clicked
void GPIOJ_IRQHandler(void){
	GPIO_PORTJ_ICR_R = 0x03; //Acknowledge flag4
	state = 1;	//Toggles the state to high
}

void spin(){
	int mod = 512/slicePoints;//this is the corresponding angle for motor, so to get 8 slices every 64th loop i will get ToF meas
	for(int i = 1, j = 0; i <= 512; i++){
		cclockwise(10);//incremeant motor by 1 step (1/512 a rev)
		if(i%mod == 0){
			GPIO_PORTN_DATA_R |= 0x02;//displacement status PN1 makes sure its on
			arr[j++] = getTOFDistance();//gets distance and stores it in array
			GPIO_PORTN_DATA_R &= ~0x02;//displacement status PN1 makes sure its off
		}
	}
}

// Main Function to handle ToF, stepper motor, and LED main logic and arithmetics
int main(void){
	//Calling all Initializer Funcs
  PLL_Init();
	SysTick_Init();
	I2C_Init();
	UART_Init();
	onboardLEDs_Init();
	PortL_Init();
	PortG_Init();
	ExternalButton_Init();
	sprintf(printf_buffer,"ToF Init\r\n");
	UART_printf(printf_buffer);
	ToF_Init();
	
	// Setting the initial x displacement as 0mm
	int x = 0;
	
	// Loop which repeats forever
	while(1) {
		if(state == 1) {
			GPIO_PORTL_DATA_R &= ~0x10;
			spin();
			state = 0; // Toggling state to 0 after stepper motor ran a complete cycle
		}
		
		else {
			if(arr[0] != -1) {//If the array is filled
				sprintf(printf_buffer,"Start\r\n");
				UART_printf(printf_buffer);
				for(int i = 0, ang = 0; i < slicePoints; i++, ang+=360/slicePoints) {//runs 8 times and increments angle by 45 degrees
						sprintf(printf_buffer,"%d %d %d\r\n", x, (int)round(arr[i]*cos(ang*PI/180)) ,(int)round(arr[i]*sin(ang*PI/180))+zOffset);//calc and send to python
						UART_printf(printf_buffer);
				}
				memset(arr, -1, sizeof arr); x+=200; // Manually Incrementing x position by 20cm each time (200mm)
				sprintf(printf_buffer,"End\r\n");
				UART_printf(printf_buffer);
			}
			GPIO_PORTL_DATA_R = 0b100000; // Turn on L5 the external LED
		}
	}
}
