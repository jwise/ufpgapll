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
	reg [9:0] period = 10'd1000;
	reg [9:0] period_min = 10'd10; /* about 10 MHz */
	reg [9:0] period_max = 10'd850; /* about 28 kHz */
	
	/*** VCO period control network ***/
	
	parameter SLEW_CENTER = {1'b1, 11'h0};
	parameter SLEW_DIV = {1'b0, {11{1'b1}}};
	
	reg [12:0] slew_cur = SLEW_CENTER;
	
	reg slew_slow = 0; /* i.e., become slower */
	reg slew_fast = 0; /* i.e., become faster */

	wire do_slew_fast = slew_cur <= (SLEW_CENTER - SLEW_DIV);
	wire do_slew_slow = slew_cur >= (SLEW_CENTER + SLEW_DIV);
	reg do_slew_fast_1a = 0;
	reg do_slew_slow_1a = 0;

	wire [9:0] period_next = period + do_slew_slow_1a - do_slew_fast_1a;
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
		
		slew_cur <= (do_slew_slow_1a | do_slew_fast_1a)
		            ? SLEW_CENTER
		            : (slew_cur + slew_slow - slew_fast);
		do_slew_slow_1a <= do_slew_slow;
		do_slew_fast_1a <= do_slew_fast;
	end
	
	/*** VCO proportional phase adjust network ***/
	
	parameter PROP_CENTER = {1'b1, 5'h0};
	parameter PROP_DIV    = 6'h2;
	
	reg [5:0] prop_cur = PROP_CENTER;
	
	wire do_phase_up = prop_cur <= (PROP_CENTER - PROP_DIV);
	wire do_phase_dn = prop_cur >= (PROP_CENTER + PROP_DIV);
	
	always @(posedge clk_50) begin
		prop_cur <= prop_cur + slew_slow - slew_fast - (do_phase_dn ? PROP_DIV : 0) + (do_phase_up ? PROP_DIV : 0);
	end

	/*** VCO ***/
	reg [15:0] ctr = 16'h0000;
	
	reg vco_out = 0;
	assign vco = vco_out;
	always @(posedge clk_50) begin
		if (ctr > period) begin
			ctr <= 0;
			vco_out <= ~vco_out;
		end else begin
			ctr <= ctr + 1 + do_phase_up - do_phase_dn;
		end
	end
	
	/*** Phase comparator network ***/
	
	reg vco_1a = 0;
	reg fb_1a = 0;
	reg up_sr = 0;
	reg down_sr = 0;
	
	always @(posedge clk_50) begin
		vco_1a <= vco;
		fb_1a <= fb;
		
		if (up_sr & down_sr) begin
			up_sr <= 0;
			down_sr <= 0;
		end else begin
			if (vco & ~vco_1a)
				down_sr <= 1;
			if (fb & ~fb_1a)
				up_sr <= 1;
		end
	end
	
	always @(*) begin
		slew_fast = up_sr & ~down_sr;
		slew_slow = down_sr & ~up_sr;
	end
	
	/*** Display logic ***/
	
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
