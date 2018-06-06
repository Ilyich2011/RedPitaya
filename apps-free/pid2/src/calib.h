/**
 * $Id: calib.h 881 2013-12-16 05:37:34Z rp_jmenart $
 *
 * @brief Red Pitaya Oscilloscope Calibration Module.
 *
 * @Author Jure Menart <juremenart@gmail.com>
 *         
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#ifndef __CALIB_H
#define __CALIB_H

#include <stdint.h>
#include "redpitaya/rp.h"



 
//typedef struct rp_osc_calib_params_s {
//    uint32_t fe_ch1_fs_g_hi;            
//    uint32_t fe_ch2_fs_g_hi;            
//    uint32_t fe_ch1_fs_g_lo;            
//    uint32_t fe_ch2_fs_g_lo;           
//    int32_t  fe_ch1_dc_offs;           
//    int32_t  fe_ch2_dc_offs;          
//    uint32_t be_ch1_fs;               
//    uint32_t be_ch2_fs;                 
//    int32_t  be_ch1_dc_offs;           
//    int32_t  be_ch2_dc_offs;           
//} rp_calib_params_t;


int rp_read_calib_params(rp_calib_params_t *calib_params);

int rp_default_calib_params(rp_calib_params_t *calib_params);

#endif //__CALIB_H
