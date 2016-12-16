#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"
#include "motor_led/advance_one_timer/e_motors.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "uart/e_uart_char.h"
#include "camera/fast_2_timer/e_poxxxx.h"
#include "motor_led/advance_one_timer/e_led.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "findRed.h"

char redbuffer[160];
int numbufferred[80];
double isRedVisible;

//custom cam picture load
void getImageRed(){
	e_poxxxx_launch_capture((char *)redbuffer);
    while(!e_poxxxx_is_img_ready()){};
}
// Image processing removes useless information
void ImageRed(){	
	long i;
	double green, red, vis;
	for(i=0; i<80; i++){
		//RGB turned into an integer value for comparison
		red = (redbuffer[2*i] & 0xF8);
		green = (((redbuffer[2*i] & 0x07) << 5) | ((redbuffer[2*i+1] & 0xE0) >> 3));
		//blue = ((redbuffer[2*i+1] & 0x1F) << 3); we don't need blue for looking for red.
		if(red > green + 35){ // red will be larger than green if there is red
			numbufferred[i] = 1;
			vis++;
		}else{
			numbufferred[i] = 0;
		}		
	}	

	//If red is visable then isRedVisable turns to true
	if(vis>15){
		isRedVisible = 1;
	}else{
		isRedVisible = 0;
	}
}

void initRed()
{
	//basic set up for camera
	e_poxxxx_init_cam();
	e_poxxxx_config_cam(0,(ARRAY_HEIGHT - 4)/2,640,4,8,4,RGB_565_MODE);
	e_poxxxx_write_cam_registers(); 
}

//Main function of follower
int findRed(void){
	double centreValue;
	
	getImageRed();
	ImageRed();
	e_led_clear();

	//Take a section of the center, this means if there is an error with one it won't effect it as a whole.
	centreValue = numbufferred[38] + numbufferred[39] + numbufferred[40] + numbufferred[41] + numbufferred[42] + numbufferred[43]; // removes stray 	
	if(centreValue > 3){ //If red is in the middle 
		e_set_led(6,1);
		return 1;
	}else if(isRedVisible){//If red isn't in the center but is visable
		e_set_led(6,1);
		return 1;
	}else{// if red isn't visible and no true values 
		return 0;
	}
}

