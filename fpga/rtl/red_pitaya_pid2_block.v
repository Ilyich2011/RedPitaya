/**
 * $Id: red_pitaya_pid2_block.v 961 2015-12-14  matej.oblak + changes by Fenske $
 *
 * @brief Red Pitaya PID controller.
 *
 * @Author Matej Oblak
 * Changes by Fenske
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
 * Proportional-integral-derivative (PID) controller.
 *
 *
 *        /---\         /---\      /-----------\
 *   IN --| - |----+--> | P | ---> | SUM & SAT | ---> OUT
 *        \---/    |    \---/      \-----------/
 *          ^      |                   ^  ^
 *          |      |    /---\          |  |
 *   set ----      +--> | I | ---------   |
 *   point         |    \---/             |
 *                 |                      |
 *                 |    /---\             |
 *                 ---> | D | ------------
 *                      \---/
 *
 *
 * Proportional-integral-derivative (PID) controller is made from three parts. 
 *
 * Error which is difference between set point and input signal is driven into
 * propotional, integral and derivative part. Each calculates its own value which
 * is then summed and saturated before given to output.
 *
 * Integral part has also separate input to reset integrator value to 0.
 * 
 */

module red_pitaya_pid2_block #(
   parameter     PSR = 12         ,
   parameter     ISR = 18         ,
   parameter     DSR = 10          
)
(
   // data
   input                 clk_i           ,  // clock
   input                 rstn_i          ,  // reset - active low
   input      [ 14-1: 0] dat_i           ,  // input data
   output     [ 14-1: 0] dat_o           ,  // output data
   output     [ 14-1: 0] meas_p_i        ,  // measure value for bar graph ----------- Fenske
   output     [ 14-1: 0] meas_i_i        ,
   output     [ 14-1: 0] meas_d_i        ,

   // settings
   input      [ 14-1: 0] set_gain_i      ,  // master-gain
   input      [ 14-1: 0] set_sp_i        ,  // set point
   input      [ 14-1: 0] set_kp_i        ,  // Kp
   input      [ 14-1: 0] set_ki_i        ,  // Ki
   input      [ 14-1: 0] set_kd_i        ,  // Kd
   input      [ 14-1: 0] set_limit_up_i  ,  // Limit if it should not be 1V  ---------- Fenske
   input      [ 14-1: 0] set_limit_low_i ,
   input      [ 14-1: 0] set_int_limit_i ,  // Integrator limit  -------------------- Fenske
   input      [ 14-1: 0] set_kii_i       ,  // Second Ki
   input                 int_rst_i       ,  // integrator reset
   input                 int_arst_i         // automatic integrator reset ----------- Fenske
);

//---------------------------------------------------------------------------------
//  Set point error calculation 

reg   [ 15-1: 0] error        ;
wire  [ 25-1: 0] dat2         ;
wire  [ 18-1: 0] error2       ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      error <= 15'h0 ;
   end
   else begin
       if ({error2[18-1],|error2[18-2:15-1]} == 2'b01) //positive overflow   --------------------- Fenske
            error <= 15'h3FFF ;                              
       else if ({error2[18-1],&error2[18-2:15-1]} == 2'b10)  //negative overflow  ---------------- Fenske  
            error <= 15'h4000 ;  
       else 
            error <= error2[15-1:0];        
   end
end

assign dat2 = (($signed(set_sp_i) - $signed(dat_i)) * $signed(set_gain_i)); // error * (gain*128)
assign error2 = $signed(dat2[25-1:8-1]); // Division: dat2/128

//---------------------------------------------------------------------------------
//  Proportional part

reg   [29-PSR-1: 0] kp_reg        ;
reg   [    14-1: 0] kp_reg2       ;  // for correct bar graph value -------------- Fenske
wire  [    33-1: 0] kp_mult2      ;  // increased original kp_mult[29:0] for checking overflow --------------- Fenske

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kp_reg  <= {29-PSR{1'b0}};
   end
   else begin
       if ({kp_mult2[33-1],|kp_mult2[33-2:29-1]} == 2'b01)  //positive overflow   --------------------- Fenske
            kp_reg <= 17'h0FFFF ;                       
       else if ({kp_mult2[33-1],&kp_mult2[33-2:29-1]} == 2'b10)  //negative overflow  ---------------- Fenske
            kp_reg <= 17'h10000 ;                       
       else 
            kp_reg <= kp_mult2[29-1:PSR] ; 
   end
end

    // Proportional part for bar graph
always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kp_reg2  <= {29-PSR{1'b0}};
   end
   else begin 
       if (({kp_mult2[33-1],|kp_mult2[33-2:29-1]} == 2'b01) || ({kp_reg[29-PSR-1],|kp_reg[29-PSR-2:14-1]} == 2'b01)) //positive overflow   --------------------- Fenske
            kp_reg2 <= 14'h1FFF ;                            
       else if (({kp_mult2[33-1],&kp_mult2[33-2:29-1]} == 2'b10) || ({kp_reg[29-PSR-1],&kp_reg[29-PSR-2:14-1]} == 2'b10))  //negative overflow  ---------------- Fenske   
            kp_reg2 <= 14'h2000 ;                      
       else 
            kp_reg2 <= kp_reg[14-1:0] ;          
   end
end

assign kp_mult2 = $signed(error) * $signed(set_kp_i)*10;  // *10 added by Fenske to achieve an output multiplied by 10
assign meas_p_i = $signed(kp_reg2); // bar graph ----------------- Fenske

//---------------------------------------------------------------------------------
//  Integrator

reg   [    29-1: 0] ki_mult       ;
wire  [    33-1: 0] int_sum       ;
reg   [    32-1: 0] int_reg       ;
wire  [32-ISR-1: 0] int_shr       ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      ki_mult  <= {29{1'b0}};
      int_reg  <= {32{1'b0}};
   end
   else begin
      ki_mult <= $signed(error) * $signed(set_ki_i) ;

      if (int_rst_i || (set_ki_i == 0)) // Fenske: added set_ki_i == 0
         int_reg <= 32'h0; // reset
      else begin // Fenske-----------------------------------------------------------------------------------           
         if (int_sum[33-1:33-2] == 2'b01) begin// positive saturation         
             if (int_arst_i) begin // automatic reset -------------------------------------- Fenske
                  int_reg <= 32'h0;
                  ki_mult <= 32'h0;
             end
             else 
                  int_reg <= 32'h7FFFFFFF; // max positive 
          end
          else if (int_sum[33-1:33-2] == 2'b10) begin// negative saturation            
             if (int_arst_i) begin // automatic reset --------------------------------------- Fenske
                  int_reg <= 32'h0;
                  ki_mult <= 32'h0;
             end
             else 
                  int_reg <= 32'h80000000; // max negative 
          end
          else begin
              if ((($signed(pid_sum2) > $signed(set_limit_up_i)) || ($signed(pid_sum2) < $signed(set_limit_low_i))) && int_arst_i) begin // limit overflow + automatic reset ------------- Fenske
                       int_reg <= 32'h0; // if auto-reset is enabled and a limit is reached: reset integrator values to 0
                       ki_mult <= 32'h0; 
              end
              else if ((($signed(int_sum[32-1:32-14]) > $signed(set_int_limit_i)) || ($signed(int_sum[32-1:32-14]) < $signed(-set_int_limit_i))) && (int_arst_i == 0)) // symmetric integrator limit  
                  int_reg <= int_reg;  // if value exceeds int_limit: don't integrate, use old value     
              else 
                  int_reg <= int_sum[32-1:0]; // use sum as it is       
          end               
       end
   end
end

assign int_sum = $signed(ki_mult) + $signed(int_reg) ;

//---------------------------------------------------------------------------------
//  Second integrator
//  Fenske ------------------------------------------------------------------------

reg   [    29-1: 0] ki_mult_2     ;
wire  [    33-1: 0] int_sum_2     ;
reg   [    32-1: 0] int_reg_2     ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      ki_mult_2  <= {29{1'b0}};
      int_reg_2  <= {32{1'b0}};
   end
   else begin
      ki_mult_2 <= ($signed(int_reg[32-1:ISR-1]) + $signed(error)) * $signed(set_kii_i) ; // + error to flatten out the 1. integrator at the 0 dB gain cross over frequency

      if (int_rst_i || (set_kii_i == 0)) // Fenske: added set_ki_i == 0
         int_reg_2 <= 32'h0; // reset
      else begin // Fenske-----------------------------------------------------------------------------------           
         if (int_sum_2[33-1:33-2] == 2'b01) begin// positive saturation         
             if (int_arst_i) begin // automatic reset -------------------------------------- Fenske
                  int_reg_2 <= 32'h0;
                  ki_mult_2 <= 32'h0;
             end
             else 
                  int_reg_2 <= 32'h7FFFFFFF; // max positive 
          end
          else if (int_sum_2[33-1:33-2] == 2'b10) begin// negative saturation            
             if (int_arst_i) begin // automatic reset --------------------------------------- Fenske
                  int_reg_2 <= 32'h0;
                  ki_mult_2 <= 32'h0;
             end
             else 
                  int_reg_2 <= 32'h80000000; // max negative 
          end
          else begin
              if ((($signed(pid_sum2) > $signed(set_limit_up_i)) || ($signed(pid_sum2) < $signed(set_limit_low_i))) && int_arst_i) begin // limit overflow + automatic reset ------------- Fenske
                       int_reg_2 <= 32'h0;
                       ki_mult_2 <= 32'h0; 
              end
              else if ((($signed(int_sum_2[32-1:32-14]) > $signed(set_int_limit_i)) || ($signed(int_sum_2[32-1:32-14]) < $signed(-set_int_limit_i))) && (int_arst_i == 0)) // symmetric integrator limit  
                  int_reg_2 <= int_reg_2;       
              else
                  int_reg_2 <= int_sum_2[32-1:0]; // use sum as it is    
          end               
       end
   end
end

assign int_sum_2 = $signed(ki_mult_2) + $signed(int_reg_2) ;

//---------------------------------------------------------------------------------
//  Choose 1. or 2. integrator

reg   [    32-1: 0] int_reg_3     ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      int_reg_3  <= {32{1'b0}};
   end
   else begin
       if (set_kii_i == 0)
           int_reg_3 <= $signed(int_reg);
       else
           int_reg_3 <= $signed(int_reg_2);
   end
end   

assign int_shr = int_reg_3[32-1:ISR] ;
assign meas_i_i = $signed(int_shr); // bar graph -------------------- Fenske

//---------------------------------------------------------------------------------
//  Derivative

wire  [    29-1: 0] kd_mult       ;
reg   [29-DSR-1: 0] kd_reg        ;
reg   [29-DSR-1: 0] kd_reg_r      ;
reg   [29-DSR  : 0] kd_reg_s      ;
wire  [    14-1: 0] kd_reg_s2     ; // Derivative output for bar graph -------------- Fenske

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kd_reg   <= {29-DSR{1'b0}};
      kd_reg_r <= {29-DSR{1'b0}};
      kd_reg_s <= {29-DSR+1{1'b0}};
   end
   else begin
      kd_reg   <= kd_mult[29-1:DSR] ;
      kd_reg_r <= kd_reg;
      kd_reg_s <= $signed(kd_reg) - $signed(kd_reg_r);
   end
end

assign kd_mult = $signed(error) * $signed(set_kd_i) ;
assign kd_reg_s2 = kd_reg_s[14-1:0];  // bar graph -------------- Fenske
assign meas_d_i = $signed(kd_reg_s2);  // bar graph -------------- Fenske

//---------------------------------------------------------------------------------
//  Sum together - saturate output

wire  [   33-1: 0] pid_sum     ; // biggest posible bit-width
reg   [   14-1: 0] pid_out     ;
wire  [   14-1: 0] pid_sum2    ; // just for checking if limit is reached ----------------- Fenske

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      pid_out    <= 14'b0 ;
   end
   else begin
       // Limitation to a certain value ------------------------------------------- added by Fenske
       if ({pid_sum[33-1],|pid_sum[33-2:13]} == 2'b01) //positive overflow 
           pid_out <= set_limit_up_i ;     
       else if ({pid_sum[33-1],&pid_sum[33-2:13]} == 2'b10) //negative overflow
           pid_out <= $signed(set_limit_low_i) ; 
       else begin
           if ($signed(pid_sum2) > $signed(set_limit_up_i)) //positive limit overflow 
               pid_out <= set_limit_up_i ;
           else if ($signed(pid_sum2) < $signed(set_limit_low_i)) //negative limit overflow 
               pid_out <= $signed(set_limit_low_i) ; 
           else 
               pid_out <= pid_sum[14-1:0] ;
       end 
   end
end

assign pid_sum = $signed(kp_reg) + $signed(int_shr) + $signed(kd_reg_s) ;
assign pid_sum2 = pid_sum[14-1:0] ;  // copy of pid_out to test if limit is reached -------------------- Fenske

assign dat_o = pid_out ;

endmodule