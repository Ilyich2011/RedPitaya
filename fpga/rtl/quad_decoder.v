/**
 * $Id: quad_decoder.v 961 2013-05 Jean P. Nicolle at fpga4fun.com & KNJN LLC $
 *
 * @brief quad_decoder.
 *
 * @Author Jean P. Nicolle at fpga4fun.com & KNJN LLC
 * Changes from Fenske
 *
 * (c) http://www.fpga4fun.com/QuadratureDecoder.html
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 *
 *
 * Quadrature decoder
 *
 * We want to implement a counter that increments or decrements according to the quadrature signals. 
 * We assume that we have available an "oversampling clock" (named "clk_i" in this page) that is faster 
 * than the quadrature signals.
 * In most cases, the "quadX" signals are not synchronous to the FPGA clock. The classical solution 
 * is to use 2 extra D flip-flops per input to avoid introducing metastability into the counter.
 */



module quad_decoder
(
   // data
   input                 clk_i           ,  // clock
   input                 rstn_i          ,  // reset - active low
   input                 quadA_i         ,  
   input                 quadB_i         ,    
   output       [14-1:0] count_o              
);

reg [2:0] quadA_delayed, quadB_delayed;
always @(posedge clk_i) quadA_delayed <= {quadA_delayed[1:0], quadA_i};
always @(posedge clk_i) quadB_delayed <= {quadB_delayed[1:0], quadB_i};

// This circuit is sometimes called a "4x decoder" because it counts 
// all the transitions of the quadrature inputs.
wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];

reg [3-1:0]  count_enable2;
reg [14-1:0] count;
always @(posedge clk_i)
begin
  if(count_enable)
  begin
    count_enable2<=count_enable2+1; // 
    if(count_enable2[3-1:0] == 3'b100) // divides counts down from 4 to 1 --------- by Fenske
      begin
        if(count_direction) count<=$signed(count)+1; else count<=$signed(count)-1;
        count_enable2 <= 0;
      end  
  end
end

assign count_o = $signed(count) ;

endmodule