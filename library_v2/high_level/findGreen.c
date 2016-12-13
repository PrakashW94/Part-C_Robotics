#include "btcom/btcom.h"

#include "motor_led/e_init_port.h"
#include "motor_led/e_epuck_ports.h"

#include "motor_led/advance_one_timer/e_motors.h"
#include "motor_led/advance_one_timer/e_agenda.h"
#include "motor_led/advance_one_timer/e_led.h"

#include "uart/e_uart_char.h"

#include "camera/fast_2_timer/e_poxxxx.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "findGreen.h"

char fbwbuffer[160];
int numbuffer[80];
double isGreenVisible;

//custom cam picture load
void getImage(){
	e_poxxxx_launch_capture((char *)fbwbuffer);
	btcomSendString( "Waiting for image to be captured. \r\n" );
    while(!e_poxxxx_is_img_ready()){};
	btcomSendString( "Finishing waiting for image to be captured. \r\n" );
}
// Image processing removes useless information
void Image(){	
	long i;
	double green, red, vis;
	for(i=0; i<80; i++){
		//RGB turned into an integer value for comparison
		red = (fbwbuffer[2*i] & 0xF8);
		green = (((fbwbuffer[2*i] & 0x07) << 5) | ((fbwbuffer[2*i+1] & 0xE0) >> 3));
		//blue = ((fbwbuffer[2*i+1] & 0x1F) << 3); we don't need blue for looking for green.
		if(green > red + 30){ // green will be larger than red if there is green
			numbuffer[i] = 1;
			vis++;
		}else{
			numbuffer[i] = 0;
		}		
	}	

	//If green is visible then isGreenVisible turns to true
	if(vis>1){
		isGreenVisible = 1;
	}else{
		isGreenVisible = 0;
	}
}

void initGreen()
{
	//basic set up for camera
	e_poxxxx_init_cam();
	e_poxxxx_config_cam(0,(ARRAY_HEIGHT - 4)/2,640,4,8,4,RGB_565_MODE);
	e_poxxxx_write_cam_registers(); 
}

//Main function of follower
int findGreen(void){
	double centreValue;
	getImage();
	Image();
	e_led_clear();
	
	//Take a section of the center, this means if there is an error with one it won't effect it as a whole.
	centreValue = numbuffer[38] + numbuffer[39] + numbuffer[40] + numbuffer[41] + numbuffer[42] + numbuffer[43]; // removes stray 	
	if(centreValue > 3){ //If green is in the middle 
		e_set_led(6,1);
		return 1;
	}else if(isGreenVisible){//If green isn't in the center but is visable
		e_set_led(6,1);
		return 1;
	}else{// if green isn't visible and no true values 
		return 0;
	}
}

