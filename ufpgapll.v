module MulDivDCM(input xtal, output clk);
	parameter div = 5;
	parameter mul = 2;
	
	wire CLKFX_BUF;
	wire GND_BIT = 0;
	BUFG CLKFX_BUFG_INST (.I(CLKFX_BUF),
				.O(clk));
	DCM_SP DCM_SP_INST (.CLKFB(GND_BIT), 
			.CLKIN(xtal), 
			.DSSEN(GND_BIT), 
			.PSCLK(GND_BIT), 
			.PSEN(GND_BIT), 
			.PSINCDEC(GND_BIT), 
			.RST(GND_BIT), 
			.CLKFX(CLKFX_BUF));
	defparam DCM_SP_INST.CLK_FEEDBACK = "NONE";
	defparam DCM_SP_INST.CLKDV_DIVIDE = 2.0;
	defparam DCM_SP_INST.CLKFX_DIVIDE = div;
	defparam DCM_SP_INST.CLKFX_MULTIPLY = mul;
	defparam DCM_SP_INST.CLKIN_DIVIDE_BY_2 = "FALSE";
	defparam DCM_SP_INST.CLKIN_PERIOD = 20.000;
	defparam DCM_SP_INST.CLKOUT_PHASE_SHIFT = "NONE";
	defparam DCM_SP_INST.DESKEW_ADJUST = "SYSTEM_SYNCHRONOUS";
	defparam DCM_SP_INST.DFS_FREQUENCY_MODE = "LOW";
	defparam DCM_SP_INST.DLL_FREQUENCY_MODE = "LOW";
	defparam DCM_SP_INST.DUTY_CYCLE_CORRECTION = "TRUE";
	defparam DCM_SP_INST.FACTORY_JF = 16'hC080;
	defparam DCM_SP_INST.PHASE_SHIFT = 0;
	defparam DCM_SP_INST.STARTUP_WAIT = "TRUE";
endmodule

module Decode7(
	input [3:0] num,
	output reg [7:0] segs);
	
	always @(*)
		case (num)
		4'h0: segs = ~8'b00111111;
		4'h1: segs = ~8'b00000110;
		4'h2: segs = ~8'b01011011;
		4'h3: segs = ~8'b01001111;
		4'h4: segs = ~8'b01100110;
		4'h5: segs = ~8'b01101101;
		4'h6: segs = ~8'b01111101;
		4'h7: segs = ~8'b00000111;
		4'h8: segs = ~8'b01111111;
		4'h9: segs = ~8'b01101111;
		4'hA: segs = ~8'b01110111;
		4'hB: segs = ~8'b01111100;
		4'hC: segs = ~8'b00111001;
		4'hD: segs = ~8'b01011110;
		4'hE: segs = ~8'b01111001;
		4'hF: segs = ~8'b01110001;
		endcase
	
endmodule

module Drive7Seg(
	input clk10,
	input [15:0] display,
	input [3:0] alive,
	output wire [7:0] cath,
	output wire [3:0] ano_out);
	
	reg [3:0] ano = 4'b1110;
	assign ano_out = ano | ~alive;

	reg [9:0] counter = 10'h0;
	
	always @(posedge clk10) begin
		counter <= counter + 10'h1;
		if (counter == 10'h0)
			ano <= {ano[0], ano[3:1]};
	end
	
	wire [3:0] num;
	Decode7 deco(num, cath);
	
	assign num = (~ano[3]) ? display[15:12] :
	             (~ano[2]) ? display[11:8] :
	             (~ano[1]) ? display[7:4] :
	                         display[3:0];
endmodule

module ufpgapll(
	input xtal,
	input [3:0] btns,
	output wire [7:0] cath,
	output wire [3:0] ano,
	output wire [3:0] led,
	
	output wire [1:0] stio,
	
	input fb_u,
	output wire vco
	);
	
	wire xbuf;
	IBUFG clkbuf(.O(xbuf), .I(xtal));
	
	wire clk_50 = xbuf;
	
	reg fb_s0 = 0, fb = 0;
	
	always @(posedge clk_50) begin
		fb_s0 <= fb_u;
		fb <= fb_s0;
	end
	
	/* "period", of course, is really period / 2. */
	reg [15:0] period = 16'd1000;
	reg [15:0] period_min = 16'd10; /* about 10 MHz */
	reg [15:0] period_max = 16'd1700; /* about 14 kHz */
	
	/*** VCO period control network ***/
	
	parameter SLEW_CENTER = 9'h100;
	parameter SLEW_DIV = 9'hC0;
	
	reg [8:0] slew_cur = SLEW_CENTER;
	
	reg slew_slow = 0; /* i.e., become slower */
	reg slew_fast = 0; /* i.e., become faster */

	wire do_slew_fast = slew_cur <= (SLEW_CENTER - SLEW_DIV);
	wire do_slew_slow = slew_cur >= (SLEW_CENTER + SLEW_DIV);

	wire [15:0] period_next = period + do_slew_slow - do_slew_fast;
//	wire [15:0] period_next = 16'd60; /* around 2.4 microseconds */
	
	assign led = {do_slew_slow, do_slew_fast, slew_slow, slew_fast};
	assign stio = {slew_slow, slew_fast};
	
	always @(posedge clk_50) begin
		if (period_next >= period_max)
			period <= period_max;
		else if (period_next <= period_min)
			period <= period_min;
		else
			period <= period_next;
		
		slew_cur <= slew_cur + slew_slow - slew_fast - (do_slew_slow ? SLEW_DIV : 0) + (do_slew_fast ? SLEW_DIV : 0);
	end

	/*** VCO & phase comparator network ***/
	reg [15:0] ctr = 16'h0000;
	
	reg vco_out = 0;
	assign vco = vco_out;
	reg fb_1a = 0;
	always @(posedge clk_50) begin
		fb_1a <= fb;
		
		if (ctr > period) begin
			ctr <= 0;
			vco_out <= ~vco_out;
			if (vco_out == 0) /* positive edge of VCO */ begin
				if (~fb) /* feedback lags behind the VCO */
					slew_slow <= 1;
				slew_fast <= 0;
			end else
				slew_slow <= 0; /* don't slew forever */
		end else begin
			ctr <= ctr + 1;
			if (fb && vco_out)
				slew_slow <= 0;
			if (fb && ~fb_1a && ~vco_out) /* feedback leads the VCO */
				slew_fast <= 1;
		end
		
//		slew_fast <= (fb ^ vco_out);
//		slew_slow <= ~(fb ^ vco_out);
	end
	
	reg [20:0] displayctr = 0;
	reg [15:0] periodlat = 0;
	always @(posedge clk_50) begin
		displayctr <= displayctr + 1;
		if (&displayctr)
			periodlat <= period;
	end
	
	wire [15:0] display =
	  btns[0] ? 16'h1234 :
	  btns[1] ? 16'h4567 :
	  btns[2] ? 16'h89AB :
	  btns[3] ? 16'hCDEF :
	            periodlat;
	wire [3:0] display_alive = 4'b1111;
	
	Drive7Seg drive(clk_50, display, display_alive, cath, ano);
endmodule
