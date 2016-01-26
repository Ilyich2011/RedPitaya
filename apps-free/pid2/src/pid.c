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

#include "pid.h"
#include "fpga_pid.h"

float dac_kp_1 = 425; // ------------------- Fenske: Value for Input == Output 
float norm_int = (1/0.477); // value to achieve a normation of Ki to: 1/ms
float norm_diff = 140.3; // value to achieve a normation of Kd to: 1ns
int dac_dc_offset[] = {-150, -105};
float max_cnt = 8191;
float old_int_limit[NUM_OF_PIDS] = {1, 1, 1, 1};

FILE *fd; // new file for saving pid-settings ------------------ Fenske
int idx; // count variable
int no_settings = NUM_OF_PIDS*PARAMS_PER_PID+2; //number of PID settings (+2 for saving the offset values too)

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
    system("cd /opt/redpitaya/www/apps/pid2/\n rw"); // should make the folder "ptb-pid" writeable ------ Fenske CHANGE for pid2!!!
    /* 	The C library function int system(const char *command) passes the command name
    	or program name specified by command to the host environment to be executed
    	by the command processor and returns after the command has been completed. */
    //  (necessary for saving the settings in a .txt-file)

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

    pid_param_t pid[NUM_OF_PIDS] = {{ 0 }};
    uint32_t ireset = 0;

    for (i = 0; i < NUM_OF_PIDS; i++) {
        /* PID enabled? */
        if (params[PID_11_ENABLE + i * PARAMS_PER_PID].value == 1) {
        	pid[i].gain = (int)((params[PID_11_GAIN + i * PARAMS_PER_PID].value)*128);  // Fenske (*128 to get integer-value)
        	pid[i].setpoint = (int)(((params[PID_11_SP + i * PARAMS_PER_PID].value)*max_cnt)); // + dac_dc_offset[i]); // Fenske
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
        }

        g_pid_reg->pid[i].gain = pid[i].gain;
        g_pid_reg->pid[i].setpoint = pid[i].setpoint;
        g_pid_reg->pid[i].kp = pid[i].kp;
        g_pid_reg->pid[i].ki = pid[i].ki;
        g_pid_reg->pid[i].kd = pid[i].kd;
        g_pid_reg->pid[i].limit_up = pid[i].limit_up; // ------------------------------------------------ Fenske
        g_pid_reg->pid[i].limit_low = pid[i].limit_low; // ---------------------------------------------- Fenske
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
    
    g_pid_reg->configuration = ireset;

    g_pid_reg->out_1_offset = (int)((params[OUT_1_OFFSET].value)*max_cnt); // ------------------- Fenske
    g_pid_reg->out_2_offset = (int)((params[OUT_2_OFFSET].value)*max_cnt); // ------------------- Fenske


    // Save settings ---------------------------------------------------------------------------- Fenske
    // (for creating files it is necessary to mount the folder on the SD-card writable! see pid_init() )
    if (params[SAVE_SETTINGS].value != 0)
    {
    	switch((int) params[SAVE_SETTINGS].value)
    	{
    		case 1: fd = fopen("/opt/redpitaya/www/apps/pid2/settings1.txt", "w"); break;
    		case 2: fd = fopen("/opt/redpitaya/www/apps/pid2/settings2.txt", "w"); break;
    		case 3: fd = fopen("/opt/redpitaya/www/apps/pid2/settings3.txt", "w"); break;
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
        }
        if(fd != NULL) {
        	for(idx = 0; idx < no_settings; idx++){
        		fscanf(fd, "%f\n", &params[PARAMS_PID_PARAMS+idx].value);
        	}
        }
        fclose(fd);
        params[LOAD_SETTINGS].value = 0;
        pid_update(&params[0]); // load saved and loaded params into fpga
    }
    return 0;
}

// Reads measure values for P, I, D and Output from FPGA
int pid_update_meas_output(rp_osc_meas_res_t *ch_meas, int channel)  // bar graph data ---------- Fenske
{
	ch_meas->p = g_pid_reg->meas[channel-1].p;
	ch_meas->i = g_pid_reg->meas[channel-1].i;
	ch_meas->d = g_pid_reg->meas[channel-1].d;
	ch_meas->o = g_pid_reg->meas[channel-1].o;

	return 0;
}
