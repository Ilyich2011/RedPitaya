/**
 * $Id: red_pitaya_pid.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya 2. MIMO PID controller.
 *
 * @Author Matej Oblak
 * Additions by Julia-Aileen Fenske
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */



/**
 * GENERAL DESCRIPTION:
 *
 * Multiple input multiple output controller.
 *
 *
 *                 /-------\       /-----------\
 *   CHA -----+--> | PID11 | ------| SUM & SAT | ---> CHA
 *            |    \-------/       \-----------/
 *            |                            ^
 *            |    /-------\               |
 *            ---> | PID21 | ----------    |
 *                 \-------/           |   |
 *                                     |   |
 *  INPUT                              |   |         OUTPUT
 *                                     |   |
 *                 /-------\           |   |
 *            ---> | PID12 | --------------
 *            |    \-------/           |    
 *            |                        ˇ
 *            |    /-------\       /-----------\
 *   CHB -----+--> | PID22 | ------| SUM & SAT | ---> CHB
 *                 \-------/       \-----------/
 *
 *
 * MIMO controller is build from four equal submodules, each can have 
 * different settings.
 *
 * Each output is sum of two controllers with different input. That sum is also
 * saturated to protect from wrapping.
 * 
 */


module red_pitaya_pid2 (
   // signals
   input                 clk_i           ,  //!< processing clock
   input                 rstn_i          ,  //!< processing reset - active low
   input      [ 14-1: 0] dat_a_i         ,  //!< input data CHA
   input      [ 14-1: 0] dat_b_i         ,  //!< input data CHB
   output     [ 14-1: 0] dat_a_o         ,  //!< output data CHA
   output     [ 14-1: 0] dat_b_o         ,  //!< output data CHB
   input      [ 14-1: 0] exp_p_dat_i     ,  //!< input data from extension port ------------- Fenske
  
   // system bus
   input      [ 32-1: 0] sys_addr        ,  //!< bus address
   input      [ 32-1: 0] sys_wdata       ,  //!< bus write data
   input      [  4-1: 0] sys_sel         ,  //!< bus write byte select
   input                 sys_wen         ,  //!< bus write enable
   input                 sys_ren         ,  //!< bus read enable
   output reg [ 32-1: 0] sys_rdata       ,  //!< bus read data
   output reg            sys_err         ,  //!< bus error indicator
   output reg            sys_ack            //!< bus acknowledge signal
);

localparam  PSR = 12         ;
localparam  ISR = 18         ;
localparam  DSR = 10         ;

//---------------------------------------------------------------------------------
//  PID 11

wire [ 14-1: 0] pid_11_out   ;
reg  [ 14-1: 0] set_11_gain  ;
reg  [ 14-1: 0] set_11_sp    ;
reg  [ 14-1: 0] set_11_kp    ;  
reg  [ 14-1: 0] set_11_ki    ;
reg  [ 14-1: 0] set_11_kd    ;
reg  [ 14-1: 0] set_11_limit_up  ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_11_limit_low ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_11_int_limit ; // Value that limits the integrator value ------ Fenske
reg  [ 14-1: 0] set_11_kii   ;     // Second Ki ----------------------------------- Fenske
reg             set_11_irst  ;
reg             set_11_airst ;     // Automatic integrator reset ------------------ Fenske

wire [ 14-1: 0] meas_11_p   ;      // Measurevalue for bar graph  ----------------- Fenske
wire [ 14-1: 0] meas_11_i   ;
wire [ 14-1: 0] meas_11_d   ;

red_pitaya_pid2_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
)
i_pid11 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_a_i        ),  // input data
  .dat_o        (  pid_11_out     ),  // output data
  .meas_p_i     (  meas_11_p      ),  // bar graph  ----------------- Fenske
  .meas_i_i     (  meas_11_i      ),
  .meas_d_i     (  meas_11_d      ),

   // settings
  .set_gain_i   (  set_11_gain    ),  // master-gain 
  .set_sp_i     (  set_11_sp      ),  // set point
  .set_kp_i     (  set_11_kp      ),  // Kp
  .set_ki_i     (  set_11_ki      ),  // Ki
  .set_kd_i     (  set_11_kd      ),  // Kd
  .set_limit_up_i   (  set_11_limit_up    ),  // Output limit
  .set_limit_low_i  (  set_11_limit_low   ),  // Output limit
  .set_int_limit_i  (  set_11_int_limit   ),  // Integrator limit 
  .set_kii_i    (  set_11_kii     ),  // Second Ki
  .int_rst_i    (  set_11_irst    ),  // integrator reset
  .int_arst_i   (  set_11_airst   )   // automatic integrator reset
);

//---------------------------------------------------------------------------------
//  PID 12

wire [ 14-1: 0] pid_12_out   ;
reg  [ 14-1: 0] set_12_gain  ;
reg  [ 14-1: 0] set_12_sp    ;
reg  [ 14-1: 0] set_12_kp    ;  
reg  [ 14-1: 0] set_12_ki    ;
reg  [ 14-1: 0] set_12_kd    ;
reg  [ 14-1: 0] set_12_limit_up  ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_12_limit_low ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_12_int_limit ; // Value that limits the integrator value ------ Fenske
reg  [ 14-1: 0] set_12_kii   ;     // Second Ki ----------------------------------- Fenske
reg             set_12_irst  ;
reg             set_12_airst ;     // Automatic integrator reset ------------------ Fenske

wire [ 14-1: 0] meas_12_p   ;      // Measurevalue for bar graph  ----------------- Fenske
wire [ 14-1: 0] meas_12_i   ;
wire [ 14-1: 0] meas_12_d   ;

red_pitaya_pid2_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
)
i_pid12 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_b_i        ),  // input data
  .dat_o        (  pid_12_out     ),  // output data
  .meas_p_i     (  meas_12_p      ),  // bar graph  ----------------- Fenske
  .meas_i_i     (  meas_12_i      ),
  .meas_d_i     (  meas_12_d      ),
  
   // settings
  .set_gain_i   (  set_12_gain    ),  // master-gain
  .set_sp_i     (  set_12_sp      ),  // set point
  .set_kp_i     (  set_12_kp      ),  // Kp
  .set_ki_i     (  set_12_ki      ),  // Ki
  .set_kd_i     (  set_12_kd      ),  // Kd
  .set_limit_up_i   (  set_12_limit_up    ),  // Output limit
  .set_limit_low_i  (  set_12_limit_low   ),  // Output limit  
  .set_int_limit_i  (  set_12_int_limit   ),  // Integrator limit
  .set_kii_i    (  set_12_kii     ),  // Second Ki  
  .int_rst_i    (  set_12_irst    ),  // integrator reset
  .int_arst_i   (  set_12_airst   )  // automatic integrator reset
);

//---------------------------------------------------------------------------------
//  PID 21

wire [ 14-1: 0] pid_21_out   ;
reg  [ 14-1: 0] set_21_gain  ;
reg  [ 14-1: 0] set_21_sp    ;
reg  [ 14-1: 0] set_21_kp    ;  
reg  [ 14-1: 0] set_21_ki    ;
reg  [ 14-1: 0] set_21_kd    ;
reg  [ 14-1: 0] set_21_limit_up  ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_21_limit_low ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_21_int_limit ; // Value that limits the integrator value ------ Fenske
reg  [ 14-1: 0] set_21_kii   ;     // Second Ki ----------------------------------- Fenske
reg             set_21_irst  ;
reg             set_21_airst ;     // Automatic integrator reset ------------------ Fenske

wire [ 14-1: 0] meas_21_p   ;      // Measurevalue for bar graph  ----------------- Fenske
wire [ 14-1: 0] meas_21_i   ;
wire [ 14-1: 0] meas_21_d   ;

red_pitaya_pid2_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
)
i_pid21 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_a_i        ),  // input data
  .dat_o        (  pid_21_out     ),  // output data
  .meas_p_i     (  meas_21_p      ),  // bar graph  ----------------- Fenske
  .meas_i_i     (  meas_21_i      ),
  .meas_d_i     (  meas_21_d      ),

   // settings
  .set_gain_i   (  set_21_gain    ),  // master-gain 
  .set_sp_i     (  set_21_sp      ),  // set point
  .set_kp_i     (  set_21_kp      ),  // Kp
  .set_ki_i     (  set_21_ki      ),  // Ki
  .set_kd_i     (  set_21_kd      ),  // Kd
  .set_limit_up_i   (  set_21_limit_up    ),  // Output limit
  .set_limit_low_i  (  set_21_limit_low   ),  // Output limit  
  .set_int_limit_i  (  set_21_int_limit   ),  // Integrator limit
  .set_kii_i    (  set_21_kii     ),  // Second Ki  
  .int_rst_i    (  set_21_irst    ),  // integrator reset
  .int_arst_i   (  set_21_airst   )   // automatic integrator reset
);

//---------------------------------------------------------------------------------
//  PID 22

wire [ 14-1: 0] pid_22_out   ;
reg  [ 14-1: 0] set_22_gain  ;
reg  [ 14-1: 0] set_22_sp    ;
reg  [ 14-1: 0] set_22_kp    ;  
reg  [ 14-1: 0] set_22_ki    ;
reg  [ 14-1: 0] set_22_kd    ;
reg  [ 14-1: 0] set_22_limit_up  ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_22_limit_low ; // Value that limits the output ---------------- Fenske 
reg  [ 14-1: 0] set_22_int_limit ; // Value that limits the integrator value ------ Fenske
reg  [ 14-1: 0] set_22_kii   ;     // Second Ki ----------------------------------- Fenske
reg             set_22_irst  ;
reg             set_22_airst ;     // Automatic integrator reset ------------------ Fenske

wire [ 14-1: 0] meas_22_p   ;      // Measurevalue for bar graph  ----------------- Fenske
wire [ 14-1: 0] meas_22_i   ;
wire [ 14-1: 0] meas_22_d   ;

red_pitaya_pid2_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
)
i_pid22 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_b_i        ),  // input data
  .dat_o        (  pid_22_out     ),  // output data
  .meas_p_i     (  meas_22_p      ),  // bar graph  ----------------- Fenske
  .meas_i_i     (  meas_22_i      ),
  .meas_d_i     (  meas_22_d      ),
  
   // settings
  .set_gain_i   (  set_22_gain    ),  // master-gain 
  .set_sp_i     (  set_22_sp      ),  // set point
  .set_kp_i     (  set_22_kp      ),  // Kp
  .set_ki_i     (  set_22_ki      ),  // Ki
  .set_kd_i     (  set_22_kd      ),  // Kd
  .set_limit_up_i   (  set_22_limit_up    ),  // Output limit
  .set_limit_low_i  (  set_22_limit_low   ),  // Output limit  
  .set_int_limit_i  (  set_22_int_limit   ),  // Integrator limit
  .set_kii_i    (  set_22_kii     ),  // Second Ki  
  .int_rst_i    (  set_22_irst    ),  // integrator reset
  .int_arst_i   (  set_22_airst   )   // automatic integrator reset 
);

//---------------------------------------------------------------------------------
//  Measure values per channel (by Fenske)

wire [ 14-1: 0] meas_1_p ;
wire [ 14-1: 0] meas_1_i ;
wire [ 14-1: 0] meas_1_d ;
wire [ 14-1: 0] meas_1_o ;

wire [ 14-1: 0] meas_2_p ;
wire [ 14-1: 0] meas_2_i ;
wire [ 14-1: 0] meas_2_d ;
wire [ 14-1: 0] meas_2_o ;

assign meas_1_p = meas_11_p + meas_12_p ;
assign meas_1_i = meas_11_i + meas_12_i ;
assign meas_1_d = meas_11_d + meas_12_d ;
assign meas_1_o = pid_11_out + pid_12_out ;

assign meas_2_p = meas_21_p + meas_22_p ;
assign meas_2_i = meas_21_i + meas_22_i ;
assign meas_2_d = meas_21_d + meas_22_d ;
assign meas_2_o = pid_21_out + pid_22_out ;

//---------------------------------------------------------------------------------
//  Sum and saturation ------------------------------------------ changed by Fenske

wire [ 15-1: 0] out_1_sum   ;
reg  [ 14-1: 0] out_1_sat   ;
wire [ 15-1: 0] out_2_sum   ;
reg  [ 14-1: 0] out_2_sat   ;
reg  [ 14-1: 0] out_1_offset    ;
reg  [ 14-1: 0] out_2_offset    ;

assign out_1_sum = $signed(pid_11_out) + $signed(pid_12_out) + $signed(out_1_offset); // 
assign out_2_sum = $signed(pid_21_out) + $signed(pid_22_out) + $signed(out_2_offset); // 

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      out_1_sat <= 14'd0 ;
      out_2_sat <= 14'd0 ;
   end
   else begin
      // saturation NEW:  result = (test) ? {if true} : {if false};
      // (sat) ? {yes: set to max/min} : {no: take current value};
      out_1_sat <= (^out_1_sum[15-1:15-2]) ? {out_1_sum[15-1], {13{~out_1_sum[15-1]}}} : out_1_sum[14-1:0];
      out_2_sat <= (^out_2_sum[15-1:15-2]) ? {out_2_sum[15-1], {13{~out_2_sum[15-1]}}} : out_2_sum[14-1:0];   
   end
end

assign dat_a_o = out_1_sat ;
assign dat_b_o = out_2_sat ;

//---------------------------------------------------------------------------------
//  Rotary encoder

wire     pb_state1   ;
wire     pb_down1    ;
wire     pb_up1      ;

pushbutton_debouncer i_rot_enc_1 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_pb_i     (  exp_p_dat_i[1] ),  // input signals
  .pb_state_o   (  pb_state1      ),  // output data
  .pb_down_o    (  pb_down1       ),  
  .pb_up_o      (  pb_up1         )
);

wire     pb_state2   ;
wire     pb_down2    ;
wire     pb_up2      ;

pushbutton_debouncer i_rot_enc_2 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_pb_i     (  exp_p_dat_i[2] ),  // input signals
  .pb_state_o   (  pb_state2      ),  // output data
  .pb_down_o    (  pb_down2       ),  
  .pb_up_o      (  pb_up2         )
);

wire [ 14-1: 0] count_rot_enc   ;

quad_decoder i_quad_dec (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .quadA_i      (  pb_state1      ),  // input signals
  .quadB_i      (  pb_state2      ),  
  .count_o      (  count_rot_enc  )   // output data
);

reg  [ 14-1: 0] rot_enc         ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      rot_enc  <= 14'h0  ;
   end
   else begin 
      if (count_rot_enc[14-1:14-2]==2'b01) // postitive sat
              rot_enc <= 0; // 14'h1FFF ;
      else if (count_rot_enc[14-1:14-2]==2'b10) // negative sat
              rot_enc <= 0; // 14'h2000 ;
           else
              rot_enc <= $signed(count_rot_enc) ;      
   end
end

//---------------------------------------------------------------------------------
//
//  System bus connection

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      set_11_gain  <= 14'd0 ;
      set_11_sp    <= 14'd0 ;
      set_11_kp    <= 14'd0 ;
      set_11_ki    <= 14'd0 ;
      set_11_kd    <= 14'd0 ;
      set_11_limit_up  <= 14'h1FFF ;  // ------------- Fenske Limit, preset to maximum
      set_11_limit_low <= 14'h2000 ;
      set_11_int_limit <= 14'h1FFF ; // ------------- Fenske Integrator limit, preset to maximum 
      set_11_kii   <= 14'd0 ;
      set_11_irst  <=  1'b1 ;
      set_11_airst <=  1'b1 ;
      
      set_12_gain  <= 14'd0 ;
      set_12_sp    <= 14'd0 ;
      set_12_kp    <= 14'd0 ;
      set_12_ki    <= 14'd0 ;
      set_12_kd    <= 14'd0 ;
      set_12_limit_up  <= 14'h1FFF ;  
      set_12_limit_low <= 14'h2000 ;
      set_12_int_limit <= 14'h1FFF ; 
      set_12_kii   <= 14'd0 ; 
      set_12_irst  <=  1'b1 ;
      set_12_airst <=  1'b1 ;
      
      set_21_gain  <= 14'd0 ;
      set_21_sp    <= 14'd0 ;
      set_21_kp    <= 14'd0 ;
      set_21_ki    <= 14'd0 ;
      set_21_kd    <= 14'd0 ;
      set_21_limit_up  <= 14'h1FFF ; 
      set_21_limit_low <= 14'h2000 ; 
      set_21_int_limit <= 14'h1FFF ; 
      set_21_kii   <= 14'd0 ;      
      set_21_irst  <=  1'b1 ;
      set_21_airst <=  1'b1 ; 
      
      set_22_gain  <= 14'd0 ;
      set_22_sp    <= 14'd0 ;
      set_22_kp    <= 14'd0 ;
      set_22_ki    <= 14'd0 ;
      set_22_kd    <= 14'd0 ;
      set_22_limit_up  <= 14'h1FFF ;  
      set_22_limit_low <= 14'h2000 ;  
      set_22_int_limit <= 14'h1FFF ;  
      set_22_kii   <= 14'd0 ;   
      set_22_irst  <=  1'b1 ;
      set_22_airst <=  1'b1 ;
   end
   else begin
      if (sys_wen) begin
         if (sys_addr[19:0]==16'h0)    {set_22_airst,set_21_airst,set_12_airst,set_11_airst,set_22_irst,set_21_irst,set_12_irst,set_11_irst} <= sys_wdata[ 8-1:0] ;
    
         if (sys_addr[19:0]==16'h10)    set_11_gain <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h14)    set_11_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h18)    set_11_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h1C)    set_11_ki  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h20)    set_11_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h24)    set_11_limit_up  <= sys_wdata[14-1:0] ;  // ------------------------------ Fenske
         if (sys_addr[19:0]==16'h28)    set_11_limit_low <= sys_wdata[14-1:0] ; 
         if (sys_addr[19:0]==16'h2C)    set_11_int_limit <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h30)    set_11_kii <= sys_wdata[14-1:0] ;
         
         if (sys_addr[19:0]==16'h40)    set_12_gain <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h44)    set_12_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h48)    set_12_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h4C)    set_12_ki  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h50)    set_12_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h54)    set_12_limit_up  <= sys_wdata[14-1:0] ;  // ------------------------------ Fenske
         if (sys_addr[19:0]==16'h58)    set_12_limit_low <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h5C)    set_12_int_limit <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h60)    set_12_kii <= sys_wdata[14-1:0] ;
         
         if (sys_addr[19:0]==16'h70)    set_21_gain <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h74)    set_21_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h78)    set_21_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h7C)    set_21_ki  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h80)    set_21_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h84)    set_21_limit_up  <= sys_wdata[14-1:0] ;  // ------------------------------ Fenske
         if (sys_addr[19:0]==16'h88)    set_21_limit_low <= sys_wdata[14-1:0] ; 
         if (sys_addr[19:0]==16'h8C)    set_21_int_limit <= sys_wdata[14-1:0] ; 
         if (sys_addr[19:0]==16'h90)    set_21_kii <= sys_wdata[14-1:0] ;
         
         if (sys_addr[19:0]==16'hA0)    set_22_gain <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hA4)    set_22_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hA8)    set_22_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hAC)    set_22_ki  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hB0)    set_22_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hB4)    set_22_limit_up  <= sys_wdata[14-1:0] ;  // ------------------------------ Fenske
         if (sys_addr[19:0]==16'hB8)    set_22_limit_low <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hBC)    set_22_int_limit <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'hC0)    set_22_kii <= sys_wdata[14-1:0] ;
         
         if (sys_addr[19:0]==16'hF4)    out_1_offset  <= sys_wdata[14-1:0] ; // -------------------- Output Offset Fenske
         if (sys_addr[19:0]==16'hF8)    out_2_offset  <= sys_wdata[14-1:0] ; 
      end
   end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
end else begin
   sys_err <= 1'b0 ;

   casez (sys_addr[19:0]) 
      20'h00 : begin sys_ack <= sys_en;          sys_rdata <= {{32- 8{1'b0}}, set_22_airst,set_21_airst,set_12_airst,set_11_airst,set_22_irst,set_21_irst,set_12_irst,set_11_irst}; end
        
      20'h10 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_gain}        ; end  // ------------------------- Fenske
      20'h14 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_sp}          ; end 
      20'h18 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_kp}          ; end 
      20'h1C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_ki}          ; end 
      20'h20 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_kd}          ; end
      20'h24 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_limit_up}    ; end  
      20'h28 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_limit_low}   ; end 
      20'h2C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_int_limit}   ; end   
      20'h30 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_11_kii}         ; end 

      20'h40 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_gain}        ; end  // ------------------------- Fenske
      20'h44 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_sp}          ; end 
      20'h48 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_kp}          ; end 
      20'h4C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_ki}          ; end 
      20'h50 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_kd}          ; end
      20'h54 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_limit_up}    ; end  
      20'h58 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_limit_low}   ; end 
      20'h5C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_int_limit}   ; end
      20'h60 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_12_kii}         ; end

      20'h70 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_gain}        ; end  // ------------------------- Fenske
      20'h74 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_sp}          ; end 
      20'h78 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_kp}          ; end 
      20'h7C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_ki}          ; end 
      20'h80 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_kd}          ; end
      20'h84 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_limit_up}    ; end  
      20'h88 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_limit_low}   ; end 
      20'h8C : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_int_limit}   ; end 
      20'h90 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_21_kii}         ; end 

      20'hA0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_gain}        ; end  // ------------------------- Fenske
      20'hA4 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_sp}          ; end 
      20'hA8 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_kp}          ; end 
      20'hAC : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_ki}          ; end 
      20'hB0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_kd}          ; end
      20'hB4 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_limit_up}    ; end  
      20'hB8 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_limit_low}   ; end 
      20'hBC : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_int_limit}   ; end
      20'hC0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, set_22_kii}         ; end 
      
      20'hD0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_1_p}           ; end  // ------------------------ bar graph Fenske
      20'hD4 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_1_i}           ; end 
      20'hD8 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_1_d}           ; end 
      20'hDC : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_1_o}           ; end
      
      20'hE0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_2_p}           ; end  // ------------------------ bar graph Fenske
      20'hE4 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_2_i}           ; end 
      20'hE8 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_2_d}           ; end 
      20'hEC : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, meas_2_o}           ; end  
             
      20'hF0 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, rot_enc}            ; end // ------------------- Rotary Encoder Fenske 
      20'hF4 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, out_1_offset}       ; end // -------------------- Output offset Fenske
      20'hF8 : begin sys_ack <= sys_en;          sys_rdata <= {{32-14{1'b0}}, out_2_offset}       ; end

     default : begin sys_ack <= sys_en;          sys_rdata <=  32'h0                              ; end
   endcase
end

endmodule