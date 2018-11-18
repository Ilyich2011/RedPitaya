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
#define LOG_PERIOD 200

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
float min_intens_threshold[] = {0, 0}; // local copie of threshold value to which the XADC value is compared to
int threshold_reached[] = {0, 0}; // flag if threshold is reached (for later)
float xadc_offset[] = {0, 0}; // offsetvalue of XADC measured at pid_init()
int old_setpoint[] = {0, 0, 0, 0}; // saves the old_setpoint value

//int wwp1 = 0;		//--Zalivako
//int wwp2 = 0;   	//--Zalivako
int locked[] = {0, 0};	//--Zalivako
int pid_enabled[] = {0, 0, 0, 0}; // Zalivako
pid_param_t pid[NUM_OF_PIDS] = {{ 0 }}; //--placed here by Zalivako
uint32_t pid_configuration = 0;
int32_t pid_out_before_relock[] = {0, 0};
int32_t real_offset[] = {0,0};
int32_t log_timer = 0;
float photodetectors_log[] = {0.0, 0.0};
int enable_log = 0;

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

int pid_init(void)
{
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
int pid_update(rp_app_params_t *params)
{
    int i;

    uint32_t ireset = 0;
    
    memset(pid, 0, NUM_OF_PIDS*sizeof(pid_param_t));

    for (i = 0; i < NUM_OF_PIDS; i++) {
        /* PID enabled? */
        if (params[PID_11_ENABLE + i * PARAMS_PER_PID].value == 1) {
        	pid[i].gain = (int)((params[PID_11_GAIN + i * PARAMS_PER_PID].value)*128);  // Fenske (*128 to get integer-value)
        	pid[i].setpoint = (int)(((params[PID_11_SP + i * PARAMS_PER_PID].value)*max_cnt)); // + dac_dc_offset[i]); // Fenske
        	old_setpoint[i] = pid[i].setpoint;
        	pid[i].kp = (int)((params[PID_11_KP + i * PARAMS_PER_PID].value)*dac_kp_1);  // Fenske
            pid[i].ki = (int)((params[PID_11_KI + i * PARAMS_PER_PID].value)*norm_int);
            pid[i].kd = (int)((params[PID_11_KD + i * PARAMS_PER_PID].value)*norm_diff);
            pid[i].limit_up = (int)((params[PID_11_LIMIT_UP + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].limit_low = (int)((params[PID_11_LIMIT_LOW + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].int_limit = (int)((params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value)*max_cnt); // Fenske
            pid[i].kii = (int)((params[PID_11_KII + i * PARAMS_PER_PID].value)*norm_int);

            if(old_int_limit[i] > params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value) { // reset integrator if int_limit is set
            	g_pid_reg->pid[i].ki = 0;
            }
            old_int_limit[i] = params[PID_11_INT_LIMIT + i * PARAMS_PER_PID].value;
        }
        else if (params[PID_11_ENABLE + i * PARAMS_PER_PID].value != 1) {  // added by Fenske (to reset the output if PID is not enabled -------
            ireset |= (1 << i);
            if ((i == 2) || (i == 0)){
            	reset_gen_offset(0);
            	real_offset[0] = 0;
            }
            if ((i == 1) || (i == 3)){
            	reset_gen_offset(1);
            	real_offset[1] = 0;
            }
        }

        g_pid_reg->pid[i].gain = pid[i].gain;
        g_pid_reg->pid[i].setpoint = pid[i].setpoint;
        g_pid_reg->pid[i].kp = pid[i].kp;
        g_pid_reg->pid[i].ki = pid[i].ki;
        g_pid_reg->pid[i].kd = pid[i].kd;
        g_pid_reg->pid[i].limit_up = pid[i].limit_up - ((i == 2) ? real_offset[0] : 0) - ((i == 1) ? real_offset[1] : 0); // ------------------------------------------------ Fenske
        g_pid_reg->pid[i].limit_low = pid[i].limit_low - ((i == 2) ? real_offset[0] : 0) - ((i == 1) ? real_offset[1] : 0); // ---------------------------------------------- Fenske
        g_pid_reg->pid[i].int_limit = pid[i].int_limit; // Fenske
        g_pid_reg->pid[i].kii = pid[i].kii;
        

        if (params[PID_11_RESET + i * PARAMS_PER_PID].value == 1) {
            ireset |= (1 << i);
        }

        /* added by Fenske for automatic Integrator reset-------------------------------------------------*/
        if (params[PID_11_ARESET + i * PARAMS_PER_PID].value == 1) {
            ireset |= (1 << (i+NUM_OF_PIDS));
        }
    }
    
    pid_configuration = ireset;
    
    g_pid_reg->configuration = ireset;

    // Offset settings go to FPGA
    g_pid_reg->out_1_offset = 0;//(int)((params[OUT_1_OFFSET].value)*max_cnt); // ------------------- Fenske
    g_pid_reg->out_2_offset = 0;//(int)((params[OUT_2_OFFSET].value)*max_cnt); // ------------------- Fenske

    // Threshold for min_intensity goes to local parameter needed by "pid_min_intensity()"
    min_intens_threshold[0] = params[MIN_I_THRESHOLD_1].value; // --------------- Fenske
    min_intens_threshold[1] = params[MIN_I_THRESHOLD_2].value; // --------------- Fenske

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
    
    pid_enabled[0] = params[PID_11_ENABLE + 0 * PARAMS_PER_PID].value;
    pid_enabled[1] = params[PID_11_ENABLE + 3 * PARAMS_PER_PID].value;
    pid_enabled[2] = params[PID_11_ENABLE + 2 * PARAMS_PER_PID].value;
    pid_enabled[3] = params[PID_11_ENABLE + 1 * PARAMS_PER_PID].value;
    
    for (i = 0; i<2; i++)
    	if (((pid_enabled[i] == 0) || (pid_enabled[i+2] == 0)) && (locked[i] > RELOCK_TIMEOUT)){
    		//stop_relocking(i);
    		locked[i] = 0;
    	}
    
    if (params[OUT_1_OFFSET].value != 0) {
    	enable_log = 1;
    	log_timer = 0;
    	photodetectors_log[0] = 0;
    	photodetectors_log[1] = 0;
    } else enable_log = 0;
    
    return 0;
}

// Reads measure values for P, I, D and Output from FPGA
int pid_update_meas_output(rp_osc_meas_res_t *ch_meas, int channel)  // bar graph data ---------- Fenske
{
	ch_meas->p = g_pid_reg->meas[channel-1].p;
	ch_meas->i = g_pid_reg->meas[channel-1].i;
	ch_meas->d = g_pid_reg->meas[channel-1].d;
	ch_meas->o = g_pid_reg->meas[channel-1].o;
	ch_meas->min_intensity = pid_min_intensity(channel);
	return 0;
}

// Reads measure value of XADC and compares it to the threshold given in the interface ------------------ Fenske
float pid_min_intensity(int channel)
{
	float intensity = 0;
	int32_t offs = 0;
	int32_t up = 0;
	int32_t low = 0;
	int channelu = channel % 2;	
	channel = channel - 1;
	
	rp_AIpinGetValue(channel+2, &intensity); // read XADC2 value
	intensity = intensity - xadc_offset[channel]; // subtract the offset
	if (pid_enabled[channel] != 0){
		if((intensity >= min_intens_threshold[channel])){ // compare the intensity to the threshold
			if ((locked[channel]>RELOCK_TIMEOUT)){
				//stopping generator and setting offset to the last generator's value
				offs = stop_relocking(channel);
				real_offset[channel] = offs;
				//correcting output limits according to the offset by generator
				g_pid_reg->pid[(channel == 0) ? 2 : 1].limit_up = pid[(channel == 0) ? 2 : 1].limit_up - real_offset[channel]; 
        			g_pid_reg->pid[(channel == 0) ? 2 : 1].limit_low = pid[(channel == 0) ? 2 : 1].limit_low - real_offset[channel];
				
				//turning integrator back on

				g_pid_reg->pid[(channel == 0) ? 2 : 1].ki = pid[(channel == 0) ? 2 : 1].ki;
				g_pid_reg->pid[(channel == 0) ? 2 : 1].kii = pid[(channel == 0) ? 2 : 1].kii;
			}
			locked[channel] = 0;
		}
		if((min_intens_threshold[channel] < 1) && (intensity < min_intens_threshold[channel])){
			if (locked[channel] == RELOCK_TIMEOUT){
			//time to turn on generator!!!
				
				pid_out_before_relock[channel] = (int32_t)(g_pid_reg->meas[channelu].o & 0x3FFF);
				if (pid_out_before_relock[channel] & (1<<13)) {
					//value is negative. We need to fill all other bits on the left with ones as yet it is signed 14-bit value
					pid_out_before_relock[channel] |= (uint32_t) (~0x3FFF);
				}

				//step one: resetting pid integrator (to prevent it from interfering in relocking proccess)
				g_pid_reg->pid[(channel == 0) ? 2 : 1].ki = 0;
				g_pid_reg->pid[(channel == 0) ? 2 : 1].kii = 0;
				//g_pid_reg->configuration = (pid_configuration | (1 <<  ((channel == 0) ? 0 : 3)));
				
				
				
				offs = real_offset[channel];
				
				up = pid[(channel == 0) ? 2 : 1].limit_up;
				low = pid[(channel == 0) ? 2 : 1].limit_low;
				
				//step three: starting generator from the initial position of the pid controller scanning all with triangles
				start_relocking_generation(channel, pid_out_before_relock[channel]+offs, low/max_cnt,up/max_cnt);
	
               			locked[channel]++;
			} 
			if (locked[channel] <= RELOCK_TIMEOUT) locked[channel]++;
		}
	}
	return intensity; // threshold_reached;
}
