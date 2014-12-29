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

module bcd(
	input [16:0] inp,
	output wire [16:0] bcd);
	
	reg [3:0] thou, hund, ten, one;
	
	integer i;
	always @(*) begin
		thou = 0;
		hund = 0;
		ten = 0;
		one = 0;
		
		for (i = 15; i >= 0; i = i - 1) begin
			if (thou >= 5)
				thou = thou + 3;
			if (hund >= 5)
				hund = hund + 3;
			if (ten >= 5)
				ten = ten + 3;
			if (one >= 5)
				one = one + 3;
			
			{thou,hund,ten,one} = {thou[2:0], hund, ten, one, inp[i]};
		end
	end
	
	assign bcd={thou,hund,ten,one};
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
	
	reg [9:0] freq = 10'd1000;
	reg [9:0] freq_min = 10'd10; /* about 10 MHz */
	reg [9:0] freq_max = 10'd2000; /* about 28 kHz */
	
	/*** VCO freq control network ***/
	
	parameter SLEW_CENTER = {1'b1, 8'h0};
	parameter SLEW_DIV = {1'b0, {8{1'b1}}};
	
	reg [12:0] slew_cur = SLEW_CENTER;
	
	reg slew_slow = 0; /* i.e., become slower */
	reg slew_fast = 0; /* i.e., become faster */

	wire do_slew_fast = slew_cur <= (SLEW_CENTER - SLEW_DIV);
	wire do_slew_slow = slew_cur >= (SLEW_CENTER + SLEW_DIV);
	reg do_slew_fast_1a = 0;
	reg do_slew_slow_1a = 0;

	wire [15:0] freq_next = freq - do_slew_slow_1a + do_slew_fast_1a;
	
	assign led = {do_slew_slow, do_slew_fast, slew_slow, slew_fast};
	assign stio = {slew_slow, slew_fast};
	
	always @(posedge clk_50) begin
		if (freq_next >= freq_max)
			freq <= freq_max;
		else if (freq_next <= freq_min)
			freq <= freq_min;
		else
			freq <= freq_next;
		
		slew_cur <= (do_slew_slow_1a | do_slew_fast_1a)
		            ? SLEW_CENTER
		            : (slew_cur + slew_slow - slew_fast);
		do_slew_slow_1a <= do_slew_slow;
		do_slew_fast_1a <= do_slew_fast;
	end
	
	/*** VCO ***/
	reg [15:0] ctr = 16'h0000;
	
	assign vco = ctr[15];
	always @(posedge clk_50) begin
		ctr <= ctr + freq + (16'h100 * slew_fast) - (16'h100 * slew_slow);
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
	
	/* frequency in KHz is freq / 2^16 * 50 000 */
	
	reg [31:0] freq_out;
	wire [15:0] freq_bcd;
	bcd bcd(freq_out[31:16], freq_bcd);
	reg [15:0] freq_bcd_1a = 0;
	reg [15:0] freq_bcd_2a = 0;
	reg [15:0] freq_bcd_3a = 0;

	
	reg [20:0] displayctr = 0;
	reg [15:0] freqlat = 0;
	always @(posedge clk_50) begin
		displayctr <= displayctr + 1;
		freq_out <= freq * 16'd50000;
		freq_bcd_1a <= freq_bcd;
		freq_bcd_2a <= freq_bcd_1a;
		freq_bcd_3a <= freq_bcd_2a;
		if (&displayctr) begin
			freqlat <= freq_bcd_3a;
		end
	end
	
	wire [15:0] display =
	  btns[0] ? 16'h1234 :
	  btns[1] ? 16'h4567 :
	  btns[2] ? 16'h89AB :
	  btns[3] ? 16'hCDEF :
	            freqlat;
	wire [3:0] display_alive =
	   { |display[15:12],
	     display_alive[3] | |display[11:8],
	     display_alive[2] | |display[7:4],
	     display_alive[1] | |display[3:0]};
	
	Drive7Seg drive(clk_50, display, display_alive, cath, ano);
endmodule
