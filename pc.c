#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>
#include <errno.h>
#include <unistd.h>

#include "dlab_def.h"

pthread_t Control_pthread;
sem_t data_avail; // Do not change the name of this semaphore

// Declare global variables (common data), for example:
#define MAXS 10000 // Maximum no of samples

float Kp = 60; 					
float run_time = 10.0; 	// Set the initial run time to 10 seconds.
float Fs = 500.0; 			// Set the initial sampling frequency to 200 Hz.
float Tf = 10;					// Set the total run time
float Tt = 0.01;
float magnitude = 120;		// Magnitude of Step/Sqaure
float frequency = 0.5;		// Square frequency
float duty_cycle = 50.0;		// Square duty cycle; percentage value

// PROPORTIONAL INTEGRAL DERIVATIVE CONTROL
int N = 20;
float Ti = 0.02625;
float Td = 0.0065625;

int no_of_samples, k;

// Increase value of MAXS for more samples
float theta[MAXS]; 			// Array for storing motor position
float ref[MAXS]; 			// Array for storing reference input
float motor_position;

float satblk(float v);

void menu() {

	printf("r:	Run the control algorithm\n");
	printf("p:	Change value of Kp\n");
	printf("f:	Change value of sample frequency, Fs\n");
	printf("t:	Change value of total run time, Tf\n");
	printf("i:	Change value of Ti\n");
	printf("d:	Change value of Td\n");
	printf("n:	Change value of N\n");
	printf("u:	Change the type of inputs (Step or Square)\n");
	printf("	 Step; prompt for the magnitude of the step\n");
	printf("	 Square; prompt for the magnitude, frequency and duty cycle\n");
	printf("g:	Plot motor position on screen\n");
	printf("h:	Save a hard copy of the plot in Postscript\n");
	printf("q:	exit\n");

}

void *Control(void *arg) {

	float e, p, i, d, u, v, a, T = 1/Fs;
	float e_prev, i_prev, d_prev, a_prev;

	e_prev = 0;
	i_prev = 0;
	d_prev = 0;
	a_prev = 0;
	k = 0;
	
	float y;

	while (k < no_of_samples) {

		sem_wait(&data_avail);					// Signal for Semaphore

		motor_position = EtoR(ReadEncoder());	// Get motor position
		//printf("%f\n", motor_position);

		// ANTI-WINDUP PID CONTROL
		e = ref[k] - motor_position;			// Calculate tracking error
		//printf("%f\n", ref);

		//printf("e = %f\n", e);
		p = Kp*e;							// Proportional Part

		i = i_prev + ((Kp/Ti)*e_prev*T) + ((1/Tt)*a_prev*T);	// Integral Part

		d = ((Td / (N*T + Td))*d_prev) + (((Kp*Td*N) / (N*T + Td))*(e - e_prev));		// Derivative Part

		v = p + i + d;				// Calculate control value/voltage

		sem_post(&data_avail);

		u = satblk(v); 					// Actuator Saturation Model

		a = u - v;						// Update Integral Component
		
		//printf("a = %f\n", a);

		e_prev = e;
		i_prev = i;
		d_prev = d;
		a_prev = a;
	
		if (DtoA(VtoD(u)) != 0) {			// Send digital code to D/A converter
			printf("Error sending digital code to D/A converter\n");
			exit(-1);
		}
		
		y = theta[k] + ref[k];
		
		theta[k] = motor_position;				// Update motor position
		
		//printf("%f\n", motor_position);
		k++;
	}

	pthread_exit(NULL);
}

float satMAX = 3.0;
float satMIN = -3.0;

float satblk(float v) {

	sem_wait(&data_avail);

	if (v > satMAX) {
		return satMAX;
	} else if (v < satMIN) {
		return satMIN;
	} else {
		return v;
	}

	sem_post(&data_avail);
	
}

int main(void *arg) {

	char selection;			// User action

	int input_type;			// Type of input: 1-Step or 2-Square
	int motor_number = 7; 	// Check your motor module for motor_number.
	int i;

	no_of_samples = (int) (Tf*Fs);
	
	menu(); 				// Print out selection menu
	
	printf("> Selection: ");
	while (1) {
	
		// Prompt user for selection
		scanf("%c", &selection);

		switch (selection) {
			case 'r':
				printf("ref = %f\n", ref);
				printf("no_of_samples = %d\n", no_of_samples);
				printf("Fs = %f\n", Fs);
				printf("Mag = %f\n", magnitude);
				printf("freq = %f\n", frequency);
				printf("DC = %f\n", duty_cycle);

				Square(ref, no_of_samples, Fs, magnitude*PI/180.0, frequency, duty_cycle);

				sem_init(&data_avail, 0, 0);		// Initialize Semaphore

				Initialize(Fs, motor_number);

				if (pthread_create(&Control_pthread, NULL, &Control, NULL) != 0 ) {
					printf("Error creating Control Thread.\n");
					exit(-1);
				}
				pthread_join(Control_pthread, NULL);

				Terminate();						// Reset MC and communication

				sem_destroy(&data_avail);			// Destroy Semaphore
				break;
			case 'u':
				printf("> Input type (1-Step, 2-Square): ");
				if (scanf("%d", &input_type) != 1) { 		// Prompt user for type of input
					printf("Error input type\n");
					exit(-1);
				}
				
				if (input_type == 1) {
					// Prompt for magnitude of the step
					printf("> Magnitude: ");
					if (scanf("%f", &magnitude) != 1) {
						printf("Error magnitude input\n");
						exit(-1);
					}
					printf("> You entered: %f\n", magnitude);
					// Set up the step reference input {ref[k]}
					for (i = 0; i < MAXS; i++) {
						ref[i] = magnitude * ((5 * PI) / 18);
						//printf("%f\n", ref[i]);
					}
				} else if (input_type == 2) {
					// Prompt for magnitude, frequency and duty cycle
					printf("> Magnitude, Frequency, Duty Cycle: ");
					if (scanf("%f %f %f", &magnitude, &frequency, &duty_cycle) != 3) {
						printf("Error magnitude, frequency, duty cycle input\n");
						exit(-1);
					}
					printf("> You entered: %f %f %f\n", magnitude, frequency, duty_cycle);

					// Set up {ref[k]} using DLaB function Square()
					//Square(ref, no_of_samples, Fs, magnitude*PI/180.0, frequency, duty_cycle);
				}

				break;
			case 'p':
				// Prompt user for new value of Kp
				printf("> Kp: ");

				if (scanf("%f", &Kp) != 1) {
					printf("Error Kp input\n");
					exit(-1);
				}

				printf("> You entered: %f\n", Kp);
				break;
			case 'f':
				// Prompt user for sample frequency, Fs
				printf("> Fs: ");

				if (scanf("%f", &Fs) != 1) {
					printf("Error Fs input\n");
					exit(-1);
				}

				printf("> You entered: %f\n", Fs);
				break;
			case 't':
				// Prompt user for total run time, Tf
				printf("> Tf: ");

				if (scanf("%f", &Tf) != 1) {
					printf("Error Tf input\n");
					exit(-1);
				}

				printf("> You entered: %f\n", Tf);
				break;
			case 'i':
				// Prompt user for integral action time constant
				printf("> Ti: ");

				if (scanf("%f", &Ti) != 1) {
					printf("Error Ti input\n");
					exit(-1);
				}

				printf("> You entered: %f\n", Ti);
				break;
			case 'd':
				// Prompt user for derivative action time constant
				printf("> Td: ");

				if (scanf("%f", &Td) != 1) {
					printf("Error Td input\n");
					exit(-1);
				}

				printf("> You entered: %f\n", Td);
				break;
			case 'n':
				// Prompt user for derivative action time constant
				printf("> N: ");

				if (scanf("%d", &N) != 1) {
					printf("Error N input\n");
					exit(-1);
				
}
				printf("> You entered: %d\n", N);
				break;
			case 'g':
				// void plot(float *v1, float *v2, float Fs, int no_of_points, int term, char *title, char *xlabel, char *ylabel)
				plot(ref, theta, Fs, no_of_samples, SCREEN, "Motor Position", "Time (ms)", "Magnitude (radians)");
				break;
			case 'h':
				printf("Saving the plot results in Postscript\n");
				plot(ref, theta, Fs, no_of_samples, PS, "Anti-Windup PID Controller", "Time (s)", "Motor Position (radians)");
				break;
			case 'q':
				printf("We are done!");
				exit(0);
			default:
				//printf("Error selection input\n");
				printf("> Selection: ");
				break;
		}
	}
}
