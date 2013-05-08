#include <bcm2835.h>
#include <stdio.h>
#include <string.h>

#define	CONTROL_VER_1

#define WINDA_M1	RPI_V2_GPIO_P1_03
#define WINDA_M2	RPI_V2_GPIO_P1_05
#define WINDA_M3	RPI_V2_GPIO_P1_07
#define WINDA_M4	RPI_V2_GPIO_P1_11
#define WINDA_DIR	RPI_V2_GPIO_P1_12
#define WINDB_M1	RPI_V2_GPIO_P1_13
#define WINDB_M2	RPI_V2_GPIO_P1_15
#define WINDB_M3	RPI_V2_GPIO_P1_16
#define WINDB_M4	RPI_V2_GPIO_P1_18
#define WINDB_DIR	RPI_V2_GPIO_P1_22


#define A				0
#define B				1

#ifdef CONTROL_VER_1
	#define STEP_MAX		4
#else
	#define STEP_MAX		8
#endif


void MotorWind_SetCurrent(uint8_t wind_index, uint8_t val)
{
	uint32_t 	port_mask_set, port_mask_clear;
	uint8_t 	m4, m3, m2, m1;
	
	// Read the selected wind (0 = A, 1 = B)
	if (wind_index == 0)
	{
		m4 = WINDA_M4;
		m3 = WINDA_M3;
		m2 = WINDA_M2;
		m1 = WINDA_M1;
	}
	else
	{
		m4 = WINDB_M4;
		m3 = WINDB_M3;
		m2 = WINDB_M2;
		m1 = WINDB_M1;
	}
	
	// Set the clear mask (all 4 bits)
	port_mask_clear = (1 << m4) | (1 << m3) | (1 << m2) | (1 << m1);
	
	// Set the set mask (from the val variable)
	port_mask_set = 0;
	if (val & 0x08)	
		port_mask_set |=  (1 << m4);
	if (val & 0x04)	
		port_mask_set |=  (1 << m3);
	if (val & 0x02)	
		port_mask_set |=  (1 << m2);
	if (val & 0x01)	
		port_mask_set |=  (1 << m1);
		
	bcm2835_gpio_write_multi(port_mask_clear, LOW); // put all pins to zero
	bcm2835_gpio_write_multi(port_mask_set, HIGH);  // put the selected bits to one
}

int help(char *command)
{
	printf("Steper motor driver based on Raspberry Pi (Version 1)\n");
	#ifdef CONTROL_VER_1
		printf("Step control version 1.\n\n");
	#else
		printf("Step control version 2.\n\n");
	#endif
	
	printf("Developed by Jesus Vasquez.\n");
	#ifdef CONTROL_VER_1
		printf("Usage: %s <MODE> <SPEED> <ACCE> <ACCE_F> <CURRENT_MAX>\n Where:\n  <MODE> 0 = 16 microsteps; 1 = 8 microsteps; 2 = 4 microsteps; 3 = 2 microsteps;\n  <SPEED> 0 = 1875 rpm; 1 = 937,5 rpm; ... ; 7 = 14,6 rpm; ... ; 15 = 0,06 rpm.\n  <ACCE> 0 = without acceleration curve; 1 = with acceleration curve.\n  <ACCE_F> = acceleration factor (1..5). and\n  <CURRENT_MAX> 0 = 3,0 A ; 1 = 2,5 A ; 2 = 2,0 A ; 3 = 1,5 A ; 4 = 1,0 A ; 5 = 0,5 A\n", command);
	#else
		printf("Usage: %s <MODE> <SPEED> <ACCE> <ACCE_F> <CURRENT_MAX>\n Where:\n  <MODE> 0 = 32 microsteps; 1 = 16 microsteps; 2 = 8 microsteps; 3 = 4 microsteps;\n  <SPEED> 0 = 937,5 rpm; 1 = 468,8 rpm; ... ; 7 = 7,32 rpm; ... ; 15 = 0,03 rpm.\n  <ACCE> 0 = without acceleration curve; 1 = with acceleration curve.\n  <ACCE_F> = acceleration factor (1..5).and\n  <CURRENT_MAX> 0 = 3,0 A ; 1 = 2,5 A ; 2 = 2,0 A ; 3 = 1,5 A ; 4 = 1,0 A ; 5 = 0,5 A\n", command);
	#endif
	return 1;
}

int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
    // Use for testing
	//bcm2835_set_debug(1);
	
	uint8_t microstep_index, step_index;
	uint8_t direction = 0;
	uint8_t mode, speed;
	uint16_t microstep_time;
	uint8_t lookuptable_delta;
	uint8_t microstep_max;
	int microstep_count, microstep_togo, microstep_input;
	uint8_t speed_var = 8;
	uint16_t delay_var, delay_var_final;
	uint16_t speedchange_count = 0;
	uint8_t use_acceleration, acceleration_factor, current_index;
	
	uint8_t dac_vals[17];
	uint8_t dac_vals_all[] = {	 0,  1,  3,  4,  6,  7,  8, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, \
								 0,  1,  2,  4,  5,  6,  7,  8,  9, 10, 11, 11, 12, 12, 13, 13, 13, \
								 0,  1,  2,  3,  4,  5,  6,  6,  7,  8,  8,  9,  9, 10, 10, 10, 10, \
								 0,  1,  1,  2,  3,  4,  4,  5,  5,  6,  6,  7,  7,  7,  8,  8,  8, \
								 0,  1,  1,  1,  2,  2,  3,  3,  4,  4,  4,  5,  5,  5,  5,  5,  5, \
								 0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3 	};

	#ifdef CONTROL_VER_1
		char *mode_str[] = {"Hexal step (3200 microstep/rev)", "Octal Step (1600 microstep/rev)", "Quarter Step (800 microstep/rev)", "Double Step (400 microstep/rev)"};
		char *speed_str[] = {"1875 rpm", "937,5 rpm", "468,75 rpm", "234,36 rpm", "117,19 rpm", "58,59 rpm", "29,30 rpm", "14,65 rpm", "7,32 rpm", "3,66 rpm", "1,83 rpm", "0,92 rpm", "0,46 rpm", "0,23 rpm", "0,11 rpm", "0,06 rpm"};
	#else
		char *mode_str[] = {"Hexal step (6400 microstep/rev)", "Octal Step (3200 microstep/rev)", "Quarter Step (1600 microstep/rev)", "Double Step (800 microstep/rev)"};
		char *speed_str[] = {"937,5 rpm", "468,75 rpm", "234,38 rpm", "117,19 rpm", "58,59 rpm", "29,30 rpm", "14,65 rpm", "7,32 rpm", "3,66 rpm", "1,83 rpm", "0,92 rpm", "0,46 rpm", "0,23 rpm", "0,11 rpm", "0,06 rpm", "0,03 rpm"};
	#endif
		char *current_str[] = {"3,0 A", "2,5 A", "2,0 A", "1,5 A", "1,0 A", "0,5 A" };
	
	if (argc != 6)
		return help(argv[0]);
	
	mode = atoi(argv[1]); 
	if (mode > 3)
		return help(argv[0]);
		
	speed = atoi(argv[2]); 
	if (speed > 15)
		return help(argv[0]);
	
	use_acceleration = atoi(argv[3]);
	if (use_acceleration > 1)
		return help(argv[0]);
		
	acceleration_factor = atoi(argv[4]); 
	if ((acceleration_factor  < 1) | (acceleration_factor  > 5))
		return help(argv[0]);
	
	current_index = atoi(argv[5]);
	if (current_index  > 5) 
		return help(argv[0]);
	
    if (!bcm2835_init())
    {
    	printf("Error during BCM2835_INIT() function\n");
		return 1;
	}

	if (use_acceleration)
	{
		if (speed < 8)
			delay_var = 256;
		else
			delay_var = (1 << speed);

		delay_var_final = (1 << speed);
	}
	else
		delay_var = (1 << speed);

	memcpy(dac_vals, dac_vals_all + 17*current_index, 17);
	
	microstep_time = 10 * delay_var * (uint16_t)(1 << mode);
	lookuptable_delta = (uint8_t)(1 << mode);
	microstep_max = (uint8_t)(1 << (4 - mode));
	
	printf("Steper motor driver based on Raspberry Pi (Version 1)\n");
	#ifdef CONTROL_VER_1
		printf("Step control version 1.\n");
	#else
		printf("Step control version 2.\n");
	#endif
	printf("Developed by Jesus Vasquez.\n");
	printf("Mode selected = %s\n", mode_str[mode]);
	printf("Speed selected = %s\n", speed_str[speed]);
	printf("Use acceleration curve = %d\n", use_acceleration);
	printf("Acceleration factor = %d\n", acceleration_factor);
	printf("Maximux current set = %s\n\n", current_str[current_index]);
	
    // Set the pin to be an output
    bcm2835_gpio_fsel(WINDA_M1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDA_M2, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDA_M3, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDA_M4, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDA_DIR, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDB_M1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDB_M2, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDB_M3, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDB_M4, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(WINDB_DIR, BCM2835_GPIO_FSEL_OUTP);

	microstep_count = 0;
	microstep_togo = 0;
		
    while (1)
    {
					
		if (microstep_count != microstep_togo)
		{
			direction = (microstep_count < microstep_togo);

			#ifdef CONTROL_VER_1
				switch(step_index)
				{
					case 0:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_index * lookuptable_delta]);
						break;
					case 1:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_index * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						break;
					case 2:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_index * lookuptable_delta]);
						break;
					case 3:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_index * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						break;
				}
			#else			
				switch(step_index)
				{
					case 0:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_max * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_index * lookuptable_delta]);
						break;
					case 1:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_max * lookuptable_delta]);
						break;
					case 2:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_index * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_max * lookuptable_delta]);
						break;
					case 3:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_max * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						break;
					case 4:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_max * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_index * lookuptable_delta]);
						break;
					case 5:
						bcm2835_gpio_write(WINDA_DIR, LOW);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_max * lookuptable_delta]);
						break;
					case 6:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_index * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[microstep_max * lookuptable_delta]);
						break;
					case 7:
						bcm2835_gpio_write(WINDA_DIR, HIGH);
						bcm2835_gpio_write(WINDB_DIR, !direction);
						MotorWind_SetCurrent(A, dac_vals[microstep_max * lookuptable_delta]);
						MotorWind_SetCurrent(B, dac_vals[(microstep_max - microstep_index) * lookuptable_delta]);
						break;
				}
			#endif		
				
			if (++microstep_index >= microstep_max)
			{
				microstep_index = 0;
				if (++step_index >= STEP_MAX)
					step_index = 0;				
			}
			
			bcm2835_delayMicroseconds(microstep_time);
			
			if(direction)
				microstep_count++;
			else
				microstep_count--;
				
			if (use_acceleration)
			{
				if (delay_var > delay_var_final)
				{
					if (++speedchange_count >= acceleration_factor)
					{
						speedchange_count = 0;
						delay_var--;
						microstep_time = 10 * delay_var * (uint16_t)(1 << mode);
					}
				}
			}

		}
		else
		{
			printf("Current position: %d\n", microstep_count);
			printf("Number of microstep to go?: ");
			scanf("%d",&microstep_input);
			microstep_togo += microstep_input;
			printf("Going to position %d ...\n\n", microstep_togo);

			if (use_acceleration)
			{
				if (speed < 8)
					delay_var = 256;
				else
					delay_var = (1 << speed);

				delay_var_final = (1 << speed);
				
				microstep_time = 10 * delay_var * (uint16_t)(1 << mode);
			}
				
		}
		
    }
    bcm2835_close();
    return 0;
}

