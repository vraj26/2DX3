#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





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
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;           // Activate clock for Port E
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {} // Allow time for clock to stabilize
    GPIO_PORTF_DIR_R |= 0b00010101;                // Enable PF4 as an output
    GPIO_PORTF_DEN_R |= 0b00010101;                     // 
    return;
}
void PortH_Init(void) {
    // Use PortM pins (PM0-PM3) for output
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;          // Activate clock for Port M
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0) {}  // Allow time for clock to stabilize
    GPIO_PORTH_DIR_R |= 0x0F;                         // Configure Port M pins (PM0-PM3) as output
    GPIO_PORTH_AFSEL_R &= ~0x0F;                      // Disable alt funct on Port M pins (PM0-PM3)
    GPIO_PORTH_DEN_R |= 0x0F;                         // Enable digital I/O on Port M pins (PM0-PM3)
                                                      // Configure Port M as GPIO
    GPIO_PORTH_AMSEL_R &= ~0x0F;                      // Disable analog functionality on Port M pins (PM0-PM3)
    return;
}
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void FlashLED(int count) {
    while(count--) {
        GPIO_PORTF_DATA_R ^= 0b00010000;
				SysTick_Wait10ms(5);      
				GPIO_PORTF_DATA_R ^= 0b00010000;	// Toggle LED
                   
    }
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
void bus_check(void){
	while(1) {
        GPIO_PORTF_DATA_R ^= 0b00000100;
				SysTick_Wait10ms(1);     
	}
}

void spin(void) {
   uint32_t delay = 1;  // Set the delay between each step
    
    // Rotate the motor in a clockwise direction by 11.25 degrees
    GPIO_PORTH_DATA_R = 0b00000011;    // Step 1
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00000110;    // Step 2
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001100;    // Step 3
    SysTick_Wait10ms(delay);
    GPIO_PORTH_DATA_R = 0b00001001;    // Step 4
    SysTick_Wait10ms(delay);
}


void returnhome(void) {
		uint32_t delay = 1; // Set the delay between each step
		// Rotate the motor counterclockwise to return to the home position
    for(int i = 0; i < 512; i++) {
        GPIO_PORTH_DATA_R = 0b00001001;    // Step 1
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00001100;    // Step 2
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000110;    // Step 3
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000011;    // Step 4
        SysTick_Wait10ms(delay);
    }
}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int stat=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	char distance_measurement;
	
	int input = 0;
	//initialize
	PLL_Init();	
	PortF_Init();
	PortH_Init();
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	//bus_check();

	// Print a message indicating the start of the program
	UART_printf("Program Begins\r\n");

	// Initialize a variable to hold a number
	int num = 1;

	// Format a message with the program code and store it in the printf_buffer
	sprintf(printf_buffer, "2DX ToF Program Studio Code %d\r\n", num);

	// Print the formatted message
	UART_printf(printf_buffer);

	// Get the sensor ID and module type
	stat = VL53L1X_GetSensorId(dev, &wordData);

	// Format a message with the sensor ID and module type and store it in the printf_buffer
	sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);

	// Print the formatted message
	UART_printf(printf_buffer);

	// Check the boot state of the ToF sensor until it is booted
	while(sensorState == 0) {
			stat = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
	}

	// Flash all LEDs to indicate the ToF chip has booted
	FlashAllLEDs();

	// Print a message indicating that the ToF chip has booted
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

	// Clear any interrupt on the ToF sensor
	stat = VL53L1X_ClearInterrupt(dev);

	// Initialize the ToF sensor
	stat = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", stat);  // Check the status of the sensor initialization

	
  stat = VL53L1X_StartRanging(dev);   

	// Loop through the scanning process 12 times
	for(int k = 0; k < 12; k++) {
			// Wait for the input 's' from UART
			while(1) {
					input = UART_InChar();
					if (input == 's')
							break;
			}
			
			// Perform 32 scans in a full rotation
			for(int i = 0; i < 32; i++) {
					// Rotate the stepper motor by 11.25 degrees
					for(int j = 0; j < 16; j++) {
							spin();
					}
					
					// Wait until the ToF sensor data is ready
					while (dataReady == 0) {
							stat = VL53L1X_CheckForDataReady(dev, &dataReady);
							// Flash an LED to indicate waiting for data
							FlashLED1(1);
							// Wait for 5 milliseconds
							VL53L1_WaitMs(dev, 5);
					}
					
					// Reset the dataReady flag
					dataReady = 0;
					
					// Get the distance measurement from the ToF sensor
					stat = VL53L1X_GetDistance(dev, &Distance);
					
					// Clear any interrupt on the ToF sensor
					stat = VL53L1X_ClearInterrupt(dev);
					
					// Format and print the distance measurement to UART
					sprintf(printf_buffer, "%u\n", Distance);
					UART_printf(printf_buffer);
					
					// Wait for 250 milliseconds
					SysTick_Wait10ms(25);
			}
			
			// Return the motor to its home position after completing 32 scans
			returnhome();
	}
}
