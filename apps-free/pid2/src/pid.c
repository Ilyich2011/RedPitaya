/**
 * @brief Red Pitaya PID Controller
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h> // for mmap
#include <fcntl.h> // for opening files

#include "redpitaya/rp.h"

#include "pid.h"
#include "fpga_pid.h"
#include "generate.h"

#define RELOCK_TIMEOUT 20
#define SLOW_PID_PERIOD 40

// Variables
void* map_ams = (void*)(-1);

float dac_kp_1 = 425; // ------------------- Fenske: Value for Input == Output 
float norm_int = (1/0.477); // value to achieve a normation of Ki to: 1/ms
float norm_diff = 140.3; // value to achieve a normation of Kd to: 1ns
int dac_dc_offset[] = {-150, -105};
float max_cnt = 8191;
float old_int_limit[NUM_OF_PIDS] = {1, 1, 1, 1}; // is needed to compare the integrator limit settings

FILE *fd; // new file for saving pid-settings ------------------ Fenske
int idx; // count variable
int no_settings = NUM_OF_PIDS*PARAMS_PER_PID+4; //number of PID settings (+4 for saving the offset and threshold values too)
float min_intens_threshold = 0; // local copie of threshold value to which the XADC value is compared to
int threshold_reached = 0; // flag if threshold is reached (for later)
float xadc_offset[] = {0, 0}; // offsetvalue of XADC measured at pid_init()
int old_setpoint[] = {0, 0, 0, 0}; // saves the old_setpoint value

//int wwp1 = 0;		//--Zalivako
//int wwp2 = 0;   	//--Zalivako
int locked = 0;	//--Zalivako
int pid_enabled = 0; // Zalivako
pid_param_t pid[NUM_OF_PIDS] = {{ 0 }}; //--placed here by Zalivako
uint32_t pid_configuration = 0;
float slow_out = 0;
int slow_enabled = 0;
short slow_counter = 0;
float slow_k = 0.0;
int slow_timeout = 0;
short relock_enabled = 0;
float retval = 0.0;

/**
 * GENERAL DESCRIPTION:
 *
 * The code below sets the PID parameters to the user-specified values.
 * A direct translation from GUI PID parameters to the FPGA registers.
 *
 * The PID Controller algorithm itself is implemented in FPGA.
 * There are 4 independent PID controllers, connecting each input (IN1, IN2)
 * to each output (OUT1, OUT2):
 *
 *                 /-------\       /-----------\
 *   IN1 -----+--> | PID11 | ------| SUM & SAT | ---> OUT1
 *            |    \-------/       \-----------/
 *            |                            ^
 *            |    /-------\               |
 *            ---> | PID21 | ----------    |
 *                 \-------/           |   |
 *                                     |   |
 *                                     |   |
 *                                     |   |
 *                 /-------\           |   |
 *            ---> | PID12 | --------------
 *            |    \-------/           |
 *            |                        ˇ
 *            |    /-------\       /-----------\
 *   IN2 -----+--> | PID22 | ------| SUM & SAT | ---> OUT2
 *                 \-------/       \-----------/
 *
 */


/*----------------------------------------------------------------------------------*/
/** @brief Initialize PID Controller module
 *
 * A function is intended to be called within application initialization. It's purpose
 * is to initialize PID Controller module and to calculate maximal voltage, which can be
 * applied on DAC device on individual channel.
 *
 * @retval     -1 failure, error message is reported on standard error
 * @retval      0 successful initialization
 */


void pid_set_me_val(int32_t val){
	retval = val;
}

int pid_init(void)
{
    int i;
    
    if(fpga_pid_init() < 0) {
        return -1;
    }
    system("cd /opt/redpitaya/www/apps/pid2\n rw"); // should make the folder "pid2" writeable ------ Fenske CHANGE for pid2!!!
    /* 	The C library function int system(const char *command) passes the command name
    	or program name specified by command to the host environment to be executed
    	by the command processor and returns after the command has been completed. */
    //  (necessary for saving the settings in a .txt-file)

    if (rp_Init() != RP_OK) { // Init API functions!!!
         fprintf(stderr, "Red Pitaya API init failed!\n");
         return EXIT_FAILURE;
    }
    // get offset value of XADCs 2 and 3 and store them to xadc_offset
    rp_AIpinGetValue(2, &xadc_offset[0]);
    rp_AIpinGetValue(3, &xadc_offset[1]);
    
    for (i = 1; i<NUM_OF_PIDS; i++){	
    	g_pid_reg->pid[i].gain = 0;
        g_pid_reg->pid[i].setpoint = 0;
        g_pid_reg->pid[i].kp = 0;
        g_pid_reg->pid[i].ki = 0;
        g_pid_reg->pid[i].kd = 0;
        g_pid_reg->pid[i].limit_up = (int)max_cnt; 
        g_pid_reg->pid[i].limit_low = (int)(-max_cnt); 
        g_pid_reg->pid[i].int_limit = (int)max_cnt;
        g_pid_reg->pid[i].kii = 0;
    }
    
    g_pid_reg->out_1_offset = 0;
    g_pid_reg->out_2_offset = 0;
    g_pid_reg->configuration = 14;
    
    return 0;
}


/*----------------------------------------------------------------------------------*/
/** @brief Cleanup PID COntroller module
 *
 * A function is intended to be called on application's termination. The main purpose
 * of this function is to release allocated resources...
 *
 * @retval      0 success, never fails.
 */
int pid_exit(void)
{
    fpga_pid_exit();

    return 0;
}


/*----------------------------------------------------------------------------------*/
/**
 * @brief Update PID Controller module towards actual settings.
 *
 * A function is intended to be called whenever one of the following settings on each PID
 * sub-controller is modified:
 *    - Enable
 *    - Integrator reset
 *    - Set-point
 *    - Kp
 *    - Ki
 *    - Kd
 *
 * @param[in] params  Pointer to overall configuration parameters
 * @retval -1 failure, error message is repoted on standard error device
 * @retval  0 succesful update
 */

int32_t zal_restore_signed_value(int32_t val){
	val = val & 0x3FFF;
	if (val & (1<<13)) {
		//value is negative. We need to fill all other bits on the left with ones as yet it is signed 14-bit value
		val |= (uint32_t) (~0x3FFF);
	}
	return val;
}

int pid_update(rp_app_params_t *params)
{
    uint32_t ireset = 0;
    short slow_reset=0, relock_reset=0, pid_reset=0; // if some of them were reset
    
    
    memset(pid, 0, NUM_OF_PIDS*sizeof(pid_param_t));
    
    //Test if some of these features were switched off
    if ((pid_enabled == 1) && (params[PID_11_ENABLE].value == 0)) pid_reset=1;
    if ((slow_enabled == 1) && (params[PID_22_ENABLE].value == 0)) slow_reset=1;
    if ((slow_enabled == 1) && (params[PID_12_ENABLE].value == 0)) relock_reset=1;
    
    slow_enabled = params[PID_22_ENABLE].value;
    relock_enabled = params[PID_12_ENABLE].value;
    slow_timeout = (int)(params[PID_22_SP].value/0.025);
    slow_k = params[PID_22_KP].value/100;
    pid[3].limit_up = (int)((params[PID_22_LIMIT_UP].value)*max_cnt);   
    pid[3].limit_low = (int)((params[PID_22_LIMIT_LOW].value)*max_cnt);
    
        /* PID enabled? */
    pid_enabled = params[PID_11_ENABLE].value;
    min_intens_threshold = params[MIN_I_THRESHOLD_1].value; 
    if (params[PID_11_ENABLE].value == 1) {
    	pid[0].gain = (int)((params[PID_11_GAIN].value)*128);  // Fenske (*128 to get integer-value)
    	pid[0].setpoint = (int)(((params[PID_11_SP].value)*max_cnt)); // + dac_dc_offset[i]); // Fenske
    	old_setpoint[0] = pid[0].setpoint;
    	pid[0].kp = (int)((params[PID_11_KP].value)*dac_kp_1);  // Fenske
        pid[0].ki = (int)((params[PID_11_KI].value)*norm_int);
        pid[0].kd = (int)((params[PID_11_KD].value)*norm_diff);
        pid[0].limit_up = (int)((params[PID_11_LIMIT_UP].value)*max_cnt); // Fenske
        pid[0].limit_low = (int)((params[PID_11_LIMIT_LOW].value)*max_cnt); // Fenske
        pid[0].int_limit = (int)((params[PID_11_INT_LIMIT].value)*max_cnt); // Fenske
        pid[0].kii = (int)((params[PID_11_KII].value)*norm_int);

        if(old_int_limit[0] > params[PID_11_INT_LIMIT].value) { // reset integrator if int_limit is set
       	    g_pid_reg->pid[0].ki = 0;
        }
        old_int_limit[0] = params[PID_11_INT_LIMIT].value;
    }
    else ireset |= 1;
    

    g_pid_reg->pid[0].gain = pid[0].gain;
    g_pid_reg->pid[0].setpoint = pid[0].setpoint;
    g_pid_reg->pid[0].kp = pid[0].kp;
    if (locked<=RELOCK_TIMEOUT)
    	g_pid_reg->pid[0].ki = pid[0].ki;
    g_pid_reg->pid[0].kd = pid[0].kd;
    g_pid_reg->pid[0].limit_up = pid[0].limit_up; // ------------------------------------------------ Fenske
    g_pid_reg->pid[0].limit_low = pid[0].limit_low; // ---------------------------------------------- Fenske
    g_pid_reg->pid[0].int_limit = pid[0].int_limit; // Fenske
    if (locked<=RELOCK_TIMEOUT)
    	g_pid_reg->pid[0].kii = pid[0].kii;
        

    if (params[PID_11_RESET].value == 1) {
        ireset |= 1;
    }

    /* added by Fenske for automatic Integrator reset-------------------------------------------------*/
    if (params[PID_11_ARESET].value == 1) {
        ireset |= (1 << NUM_OF_PIDS);
    }
    
    g_pid_reg->configuration = ireset;

    //in case if it was relocking when relock mechanism was turned off. We just leave slow out where it was
    if ((pid_enabled == 1)&&(slow_enabled == 1)&&(relock_reset == 1)&&(locked>RELOCK_TIMEOUT)){
    	locked = 0;
    	slow_counter = 0;
    	slow_out = zal_restore_signed_value(stop_relocking(1));			
	g_pid_reg->pid[0].ki = pid[0].ki;
	g_pid_reg->pid[0].kii = pid[0].kii;
    }
    //in case if slow output or PID control were turned off. Set slow out to zero.
    if ((slow_reset == 1)||(pid_reset == 1)){
    	set_channel_output(1, 0);
    	slow_out = 0;
    	slow_counter = 0;
    	locked = 0;
    	g_pid_reg->pid[0].ki = pid[0].ki;
	g_pid_reg->pid[0].kii = pid[0].kii;
    }


    // Save settings ---------------------------------------------------------------------------- Fenske
    // (for creating files it is necessary to mount the folder on the SD-card writable! see pid_init() )
    if (params[SAVE_SETTINGS].value != 0)
    {
    	switch((int) params[SAVE_SETTINGS].value)
    	{
    		case 1: fd = fopen("/opt/redpitaya/www/apps/pid2/settings1.txt", "w"); break;
    		case 2: fd = fopen("/opt/redpitaya/www/apps/pid2/settings2.txt", "w"); break;
    		case 3: fd = fopen("/opt/redpitaya/www/apps/pid2/settings3.txt", "w"); break;
    		default: break;
    	}
    	if(fd != NULL) {
    		for(idx = 0; idx < no_settings; idx++){
    			fprintf(fd, "%f\n", params[PARAMS_PID_PARAMS+idx].value);
    		}
    	}
    	fclose(fd);
    	params[SAVE_SETTINGS].value = 0;
    }
    // Load settings ---------------------------------------------------------------------------- Fenske
    if (params[LOAD_SETTINGS].value != 0)
    {
        switch((int) params[LOAD_SETTINGS].value)
        {
        	case 1: fd = fopen("/opt/redpitaya/www/apps/pid2/settings1.txt", "r"); break;
        	case 2: fd = fopen("/opt/redpitaya/www/apps/pid2/settings2.txt", "r"); break;
        	case 3: fd = fopen("/opt/redpitaya/www/apps/pid2/settings3.txt", "r"); break;
        	default: break;
        }
        if(fd != NULL) {
        	for(idx = 0; idx < no_settings; idx++){
        		fscanf(fd, "%f\n", &params[PARAMS_PID_PARAMS+idx].value);
        	}
        }
        fclose(fd);
        params[LOAD_SETTINGS].value = 0;
        pid_update(&params[0]); // load saved and loaded params into fpga
        
        //This is for power logging
        //We should dump the file here
        fd = fopen("/opt/redpitaya/www/apps/pid2/power_log.txt", "w");
        fclose(fd);
        
    }    
    
    return 0;
}

// Reads measure values for P, I, D and Output from FPGA
int pid_update_meas_output(rp_osc_meas_res_t *ch_meas, int channel)  // bar graph data ---------- Fenske
{
	if (channel == 1){
		ch_meas->p = g_pid_reg->meas[channel-1].p;
		ch_meas->i = g_pid_reg->meas[channel-1].i;
		ch_meas->d = g_pid_reg->meas[channel-1].d;
		ch_meas->o = g_pid_reg->meas[channel-1].o;
		ch_meas->min_intensity = pid_min_intensity(channel);
	} else {
		ch_meas->p = 0;
		ch_meas->i = 0;
		ch_meas->d = 0;
		ch_meas->o = ((int) slow_out) & 0x3FFF;
		ch_meas->min_intensity = 0;
		pid_min_intensity(channel);
	}
	return 0;
}

// Reads measure value of XADC and compares it to the threshold given in the interface ------------------ Fenske
float pid_min_intensity(int channel)
{
	float intensity = 0;
	int32_t offs = 0;
	int32_t up = 0;
	int32_t low = 0;
	channel = channel - 1;
	rp_AIpinGetValue(channel+2, &intensity); // read XADC2 value
	intensity = intensity - xadc_offset[channel]; // subtract the offset
	if (channel == 1) return intensity;
	
	
	if ((pid_enabled != 0) && (slow_enabled == 1) && (relock_enabled == 1)){
		if(intensity >= min_intens_threshold){ // compare the intensity to the threshold
			if (locked>RELOCK_TIMEOUT){
				//stopping generator and setting offset to the last generator's value
				slow_out = zal_restore_signed_value(stop_relocking(1));
				
				//turning integrator back on
				g_pid_reg->pid[0].ki = pid[0].ki;
				g_pid_reg->pid[0].kii = pid[0].kii;
				
			}
			locked = 0;
		}
		if(intensity < min_intens_threshold){
			if (locked == RELOCK_TIMEOUT){
			//time to turn on generator!!!
				//step one: resetting pid integrator (to prevent it from interfering in relocking proccess)
				g_pid_reg->pid[0].ki = 0;
				g_pid_reg->pid[0].kii = 0;
				//g_pid_reg->configuration = (pid_configuration | (1 <<  ((channel == 0) ? 0 : 3)));
				
				up = pid[3].limit_up;
				low = pid[3].limit_low;
				
				//step three: starting generator from the initial position of the pid controller scanning all with triangles
				start_relocking_generation(1, slow_out, low/(max_cnt+1),up/(max_cnt+1));

               			locked++;
               			slow_counter = 0;
			} 
			if (locked <= RELOCK_TIMEOUT) locked++;
		}
	}
	
	if ((pid_enabled == 1) && (slow_enabled == 1) && (locked<=RELOCK_TIMEOUT)){
		if (slow_counter >= slow_timeout) {
			slow_counter = 0;
			offs = zal_restore_signed_value(g_pid_reg->meas[0].i);
			slow_out = slow_out + slow_k*offs;
			retval = zal_restore_signed_value(pid[3].limit_low);
			if (slow_out>pid[3].limit_up) slow_out = pid[3].limit_up;
			if (slow_out<pid[3].limit_low) slow_out = pid[3].limit_low;
			set_channel_output(1, slow_out);
		} else slow_counter++;
	}
	
	
	return intensity; // threshold_reached;
}
