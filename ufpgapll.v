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

	reg [12:0] counter = 0;
	
	always @(posedge clk10) begin
		counter <= counter + 1;
		if (counter == 0)
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
	input [15:0] inp,
	output wire [15:0] bcd);
	
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
	input [3:0] sw,
	output wire [7:0] cath,
	output wire [3:0] ano,
	output wire [3:0] led,
	
	output wire [1:0] stio,
	
	input fb_u,
	output wire pllout
	);
	
	wire xbuf;
	IBUFG clkbuf(.O(xbuf), .I(xtal));
	
	wire clk_50 = xbuf;
	
	reg fb_s0 = 0, fb = 0;
	
	always @(posedge clk_50) begin
		fb_s0 <= fb_u;
		fb <= fb_s0;
	end
	
	parameter FREQ_MIN = 64'd50000; /* Hz */
	parameter FREQ_DEFAULT = 64'd125000; /* Hz */
	parameter FREQ_MAX = 64'd400000; /* Hz */
	
	/* number of cycles of no input signal before PLL frequency adjustment is disabled */
	parameter NOSIG_LOCKOUT = 64'd50000000 / FREQ_MIN * 32'd2;
	
	parameter FREQ_DEFAULT_RAW = FREQ_DEFAULT * 64'd65536 / 64'd50000000;
	parameter FREQ_MIN_RAW = FREQ_MIN * 64'd65536 / 64'd50000000;
	parameter FREQ_MAX_RAW = FREQ_MAX * 64'd65536 / 64'd50000000;
	
	/*** VCO freq control network ***/

	reg [9:0] freq = FREQ_DEFAULT_RAW[9:0];
	reg [9:0] freq_min = FREQ_MIN_RAW[9:0];
	reg [9:0] freq_max = FREQ_MAX_RAW[9:0];
	
	parameter SLEW_CENTER = {1'b1, 9'h0};
	parameter SLEW_DIV = {1'b0, {7{1'b1}}, 2'b0};
	
	reg [9:0] slew_cur = SLEW_CENTER;
	
	reg slew_slow = 0; /* i.e., become slower */
	reg slew_fast = 0; /* i.e., become faster */

	reg do_slew_fast_1a = 0;
	reg do_slew_slow_1a = 0;
	wire do_slew_fast = (slew_cur <= (SLEW_CENTER - SLEW_DIV)) & ~do_slew_fast_1a;
	wire do_slew_slow = (slew_cur >= (SLEW_CENTER + SLEW_DIV)) & ~do_slew_slow_1a;
	
	wire lockout;

	wire [9:0] freq_next = freq - {9'b0,do_slew_slow_1a} + {9'b0,do_slew_fast_1a};
	
	assign stio = {slew_slow, slew_fast};
	
	always @(posedge clk_50) begin
		if (btns[2])
			freq <= FREQ_DEFAULT_RAW[9:0];
		else if (lockout)
			freq <= freq;
		else if (freq_next >= freq_max)
			freq <= freq_max;
		else if (freq_next <= freq_min)
			freq <= freq_min;
		else
			freq <= freq_next;
		
		slew_cur <= (do_slew_slow_1a | do_slew_fast_1a)
		            ? SLEW_CENTER
		            : (slew_cur + {9'b0,slew_slow} - {9'b0,slew_fast});
		do_slew_slow_1a <= do_slew_slow;
		do_slew_fast_1a <= do_slew_fast;
	end
	
	/*** Frequency lockout control logic ***/
	reg [15:0] lockout_ctr = 0;
	reg last_fb = 0;
	
	assign lockout = (lockout_ctr == NOSIG_LOCKOUT[15:0]);
	
	always @(posedge clk_50) begin
		last_fb <= fb;
		
		if (fb ^ last_fb)
			lockout_ctr <= 0;
		else if (lockout_ctr < NOSIG_LOCKOUT[15:0])
			lockout_ctr <= lockout_ctr + 1;
	end
	
	/*** VCO ***/
	reg [15:0] ctr = 16'h0000;
	
	wire vco = ctr[15];
	always @(posedge clk_50) begin
		ctr <= ctr + {6'b0,freq} + (({6'b0,freq} / 2) * slew_fast) - (({6'b0,freq} / 2) * slew_slow);
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
	
	/*** Phase offset logic ***/
	
	reg [15:0] shift_small = 0;
	wire [1:0] shift_large = sw[1:0];
	
	wire [15:0] ctr_ofs = ctr - shift_small - {shift_large,14'b0};
	reg ctr_ofs_l = 0;
	assign pllout = ctr_ofs_l;
	
	always @(posedge clk_50)
		ctr_ofs_l <= ctr_ofs[15]; /* avoid glitches */
	
	reg [15:0] pha_ctr;
	always @(posedge clk_50) begin
		pha_ctr <= pha_ctr + 1;
		
		if (&pha_ctr) begin
			if (btns[0] & btns[1])
				shift_small <= 0;
			else if (btns[0])
				shift_small <= shift_small + 1;
			else if (btns[1])
				shift_small <= shift_small - 1;
		end
	end

	/*** Display logic ***/
	
	/* frequency in KHz is freq / 2^16 * 50 000 */
	
	reg [31:0] freq_out;
	wire [15:0] shift_amt = -(shift_small + {shift_large, 14'b0});
	reg [31:0] shift_out;
	wire [15:0] freq_bcd;
	bcd fbcd(freq_out[31:16], freq_bcd);
	reg [15:0] freq_bcd_1a = 0;
	reg [15:0] freq_bcd_2a = 0;
	reg [15:0] freq_bcd_3a = 0;

	wire [15:0] shift_bcd;
	bcd sbcd(shift_out[31:16], shift_bcd);
	reg [15:0] shift_bcd_1a = 0;
	reg [15:0] shift_bcd_2a = 0;
	reg [15:0] shift_bcd_3a = 0;
	
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
		
		shift_out <= shift_amt * 16'd360;
		shift_bcd_1a <= shift_bcd;
		shift_bcd_2a <= shift_bcd_1a;
		shift_bcd_3a <= shift_bcd_2a;
	end
	
	assign led = {lockout, 1'b0, slew_slow, slew_fast};
	
	wire [15:0] display =
	  btns[0] ? shift_bcd_3a :
	  btns[1] ? shift_bcd_3a :
	  btns[2] ? {6'b0,freq_min} :
	  btns[3] ? {6'b0,freq_max} :
	            freqlat;
	wire [3:0] display_alive = 
	   { |display[15:12],
	     |display[15:8],
	     |display[15:4],
	     1'b1
	   };
	
	Drive7Seg drive(clk_50, display, display_alive, cath, ano);
endmodule
