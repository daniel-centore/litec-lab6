
/**
 * Names: Kyle Ritchie, Emmett Hitz, Daniel Centore, Adam Stanczyk
 * Section: 4A
 * Date: 2015
 * Filename: lab6.c
 * Description: TODO
 */

#include <c8051_SDCC.h>// include files. This file is available online
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

// Pulsewidth constants
#define PW_MIN 2000
#define PW_NEUT 2750
#define PW_MAX 3500

#define PCA_START 28672

#define RANGE_MAX (60)
#define RANGE_MIN (10)
#define SPEED_MAX (70)

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void Interrupt_Init(void); 	
void PCA_Init (void);
void XBR0_Init(void);
void SMB0_Init(void);
void ADC_Init(void);
void Pick_Heading(void);
void Adjust_Wheels(void);
void Drive_Motor(void);
unsigned int Read_Compass(void);
unsigned int Read_Ranger(void);
unsigned char read_AD_input(unsigned char n);
void Ping_Ranger(void);
void PCA_ISR(void) __interrupt 9;
void Update_Battery(void);
void Pick_Gains(void);
void Process(void);
unsigned int Read_Compass(void);
void Paused_LCD(void);
void Update_LCD(void);
void Steering_Goal(void);
void printDebug(void);
void updatePWM();
void Slow_Down(void);
void Correct_Heading(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

unsigned char battery_level = 0;          // Battery voltage in volts

unsigned int thrust_pw = PW_NEUT;

unsigned int current_range = 0;           // The most recent ranger distance (cm)

unsigned int wait = 0;                    // Elapsed 20ms ticks

unsigned char new_range_flag = 0;         // Flag to indicate new range available (80ms)
unsigned char new_battery_flag = 0;       // Flag to indicate new battery voltage (1s)
unsigned char new_LCD_flag = 0;           // Flag to indicate we should update the LCD (400ms)

unsigned char r_count = 0;                // overflow count for range
unsigned char b_count = 0;                // overflow count for battery reading
unsigned char l_count = 0;                // overflow count for LCD reading

signed int current_heading = 0;
signed int previous_heading = 0;

signed int nominal_heading = 0;
signed int desired_heading = 0;

signed int delta_heading = 0;

signed int error = 0;
signed int previous_error = 0;

unsigned char kP = 0;
unsigned char kD = 0;

__sbit __at 0xB0 USE_RANGER;

/********************************************************************/

void main()
{
	Sys_Init(); // Initialize the C8051 board
	putchar(' '); // Required for output to terminal
	Port_Init(); 
	PCA_Init();
	ADC_Init();
	XBR0_Init();
	SMB0_Init();
	
	printf("\r\nSTART\r\n");
	
	// Wait for ADC and motors to be ready
	while (wait < 50);
	
	PCA0CP0 = 0xFFFF - 2750;
	
	wait = 0;
	while (wait < 20);
	
	// Select heading and steering gain from keypad
	Pick_Heading();
	Pick_Gains();
	
	printf("kP*10: %d kD:%d\r\n", kP, kD);
	
	// Run main loop
	while (1)
		Process();
}

void Process()
{
	// If there's a new range available, read it and update the current value.
	// Also, update drive speed and actual steering
	// Every 80ms
	if (new_range_flag)
	{
		current_heading = Read_Compass();
		
		delta_heading = current_heading - previous_heading;
		
		if (delta_heading > 1800)
			delta_heading -= 3600;
		else if (delta_heading < -1800)
			delta_heading += 3600;
			
		//printf("%d\r\n", delta_heading);
		
		previous_heading = current_heading;
		
		if (abs(delta_heading) > SPEED_MAX)
			Slow_Down();
		else if (abs(delta_heading) != 0)
			Correct_Heading();
		
		// Read ranger value and ping again
		current_range = Read_Ranger();
		Ping_Ranger();
		
		printDebug();
		
		new_range_flag = 0;
	}
	
	// Update battery and speed from ADC (every 1s)
	if (new_battery_flag)
	{
		Update_Battery();
		
		new_battery_flag = 0;
	}
	
	// Update LCD info every 400ms
	if (new_LCD_flag)
	{
		Update_LCD();
		new_LCD_flag = 0;
	}
}

void Slow_Down(void)
{
	//printf("Slow Down\r\n");
	if (delta_heading < 0)
		thrust_pw = (PW_MAX);
	else
		thrust_pw = (PW_MIN);
	
	updatePWM();
}

void Correct_Heading(void)
{
	unsigned long pw;
	//printf("Correct Heading\r\n");
	
	if (current_range < RANGE_MIN)
		current_range = RANGE_MIN;
	else if (current_range > RANGE_MAX)
		current_range = RANGE_MAX;
	
	// TODO WTF IS GOING ON HERE
	
	if (USE_RANGER)
		desired_heading = nominal_heading + (long) (current_range - RANGE_MIN) * 3600 / (long) (RANGE_MAX - RANGE_MIN) - 1800;
	else
		desired_heading = nominal_heading;
	
	//printf("Des: %d Rang: %d \r\n", desired_heading, current_range);
	
	error = desired_heading - current_heading;
	
	if (error > 1800)
		error -= 3600;
	else if (error < -1800)
		error += 3600;
		
	pw = (long) PW_NEUT + (long) kP * (long) error / 10 + (long) kD * (long) (error - previous_error);
	
	if (pw < PW_MIN)
		pw = PW_MIN;
	else if (pw > PW_MAX)
		pw = PW_MAX;
		
	thrust_pw = pw;
	
	updatePWM();
	
	previous_error = error;
}

void updatePWM()
{
	PCA0CP0 = 0xFFFF - thrust_pw;
}

// Print "Car ready" to the LCD
void Paused_LCD(void)
{
	lcd_clear();
	lcd_print("Car Ready\n");
}

// Update LCD with current info
void Update_LCD(void)
{
	lcd_clear();
	lcd_print("Heading: %d\n", current_heading / 10);
	lcd_print("PW %d\n", thrust_pw);
	lcd_print("Goal %d\n", desired_heading / 10);
	lcd_print("Battery: %d\n", 15 * battery_level / 244);
}

// Asks the user for the desired heading
void Pick_Heading(void)
{
	do {
		lcd_clear();
		lcd_print("Heading (0-360):");
			
		nominal_heading = kpd_input(1) * 10;
	} while (desired_heading > 3600);
}

// Asks user for gains
void Pick_Gains(void)
{
	lcd_clear();
	lcd_print("kP*10:");
	kP = kpd_input(1);
	
	lcd_clear();
	lcd_print("kD:");
	kD = kpd_input(1);
}

void printDebug(void)
{
	printf("%d, %d, %d\r\n"
			, wait * 20
			, desired_heading
			, current_heading
		);
}

// Initialize ports
void Port_Init(void)
{
	P0MDOUT = 0xFF;
	
	P1MDOUT = 0x0F;
	P1 |= ~0x0F;
	P1MDIN = 0x3F;

	P3MDOUT = 0x00;
	P3 = 0xFF;
}

// Initialize PCS
void PCA_Init (void)
{
	PCA0MD = 0x81;
	PCA0CPM0 = 0xC2; // 16 bit, enable compare, enable PWM
	PCA0CPM2 = 0xC2;
	EIE1 = 0x08;
	PCA0CN |= 0x40;
	EA = 1;
}

// Initialize crossbar
void XBR0_Init(void)
{
	XBR0 = 0x27;
}

// Initialize SMB
void SMB0_Init(void)
{
	SMB0CR = 0x93;
	ENSMB = 1;
}

// Initialize ADC
void ADC_Init(void)
{
	REF0CN = 0x03;
	ADC1CF |= 0x01;
	ADC1CN = 0x80;
	AMX1SL = 5; // Set P1.n as the analog input for ADC1
}

// Read current compass heading
unsigned int Read_Compass(void)
{
	unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	unsigned char Data[2]; // Data is an array with a length of 2
	unsigned int heading; // the heading returned in degrees between 0 and 3599
	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
	heading =(((unsigned int) Data[0] << 8) | Data[1]); //combine the two values
	//heading has units of 1/10 of a degree 
	return heading; // the heading returned in degrees between 0 and 3599
}

// Read current ranger heading
unsigned int Read_Ranger(void)
{
	unsigned char Data[2];
	unsigned int range = 0;
	// the address of the ranger is 0xE0
	unsigned char addr = 0xE0;
	i2c_read_data(addr, 2, Data, 2); // read two bytes, starting at reg 2
	range = (((unsigned int) Data[0] << 8) | Data[1]);
	
	return range;
}

// Instruct ranger to send a ping
void Ping_Ranger(void)
{	
	// write 0x51 to reg 0 of the ranger:
	unsigned char Data[1];
	unsigned char addr = 0xE0;
	Data[0] = 0x51;
	i2c_write_data(addr, 0, Data , 1) ; // write one byte of data to reg 0 at addr
}

// Reads current battery 0-255 from ADC
void Update_Battery(void)
{
	ADC1CN = ADC1CN & ~0x20; // Clear the “Conversion Completed” flag
	ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);// Wait for conversion to complete
	battery_level = ADC1; // Return digital value in ADC1 register
}

void PCA_ISR(void) __interrupt 9
{
	++wait;
	
	if (CF)
	{
		CF = 0;
		PCA0 = PCA_START;
		
		++r_count;
		if (r_count >= 4)		// 80 ms
		{
			new_range_flag = 1;
			r_count = 0;
		}
		
		++l_count;
		if (l_count >= 20)		// 400 ms
		{
			new_LCD_flag = 1;
			l_count = 0;
		}
		
		++b_count;
		if (b_count >= 50)		// 1 second
		{
			new_battery_flag = 1;
			b_count = 0;
		}
	}

	PCA0CN &= 0xC0;
}
