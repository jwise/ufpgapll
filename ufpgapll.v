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
	input decimal,
	output reg [7:0] segs);
	
	always @(*)
		case (num)
		4'h0: segs = ~{decimal, 7'b0111111};
		4'h1: segs = ~{decimal, 7'b0000110};
		4'h2: segs = ~{decimal, 7'b1011011};
		4'h3: segs = ~{decimal, 7'b1001111};
		4'h4: segs = ~{decimal, 7'b1100110};
		4'h5: segs = ~{decimal, 7'b1101101};
		4'h6: segs = ~{decimal, 7'b1111101};
		4'h7: segs = ~{decimal, 7'b0000111};
		4'h8: segs = ~{decimal, 7'b1111111};
		4'h9: segs = ~{decimal, 7'b1101111};
		4'hA: segs = ~{decimal, 7'b1110111};
		4'hB: segs = ~{decimal, 7'b1111100};
		4'hC: segs = ~{decimal, 7'b0111001};
		4'hD: segs = ~{decimal, 7'b1011110};
		4'hE: segs = ~{decimal, 7'b1111001};
		4'hF: segs = ~{decimal, 7'b1110001};
		endcase
	
endmodule

module Drive7Seg(
	input clk10,
	input [15:0] display,
	input [3:0] decimal,
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
	wire cdecimal;
	Decode7 deco(num, cdecimal, cath);
	
	assign num = (~ano[3]) ? display[15:12] :
	             (~ano[2]) ? display[11:8] :
	             (~ano[1]) ? display[7:4] :
	                         display[3:0];
	assign cdecimal = (~ano[3]) ? decimal[3] :
	                  (~ano[2]) ? decimal[2] :
	                  (~ano[1]) ? decimal[1] :
	                              decimal[0];
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
	
	/*** Tunables ***/
	
	parameter XTAL_FREQ = 64'd50000000; /* Hz */

	parameter FREQ_LOCKOUT_LO = 64'd50000; /* Hz */
	parameter FREQ_MIN = 64'd100000; /* Hz */
	parameter FREQ_DEFAULT = 64'd125000; /* Hz */
	parameter FREQ_MAX = 64'd200000; /* Hz */
	parameter FREQ_LOCKOUT_HI = 64'd300000; /* Hz */
	
	parameter LOCKOUT_TIME = 64'd100000; /* ns */

	parameter SLEW_RATE = 13; /* ns / Hz -- n.b.: 13 is upper limit in 9 bits */
	
	/* XXX: should at least calculate phase rate, but that is a huge pain to compute: it is in degrees / (VCO Hz * ns) or so */
	
	/*** Internal clock generation ***/
	
	wire xbuf;
	IBUFG clkbuf(.O(xbuf), .I(xtal));
	
	wire clk_50 = xbuf;
	
	/*** Feedback synchronization network ***/
	
	reg fb_s0 = 0, fb = 0;
	
	always @(posedge clk_50) begin
		fb_s0 <= fb_u;
		fb <= fb_s0;
	end
	
	/*** Computed parameters ***/
	
	parameter XTAL_NS = 64'd1000000000 / XTAL_FREQ; /* ns / clock */
	
	/* number of cycles of no input signal before PLL frequency adjustment is disabled */
	parameter CYCLES_LOCKOUT_LO = XTAL_FREQ / (FREQ_LOCKOUT_LO * 64'd2);
	parameter CYCLES_LOCKOUT_HI = XTAL_FREQ / (FREQ_LOCKOUT_HI * 64'd2);
	parameter CYCLES_LOCKOUT = LOCKOUT_TIME / XTAL_NS;
	
	parameter FREQ_DEFAULT_RAW = FREQ_DEFAULT * 64'd65536 / XTAL_FREQ;
	parameter FREQ_MIN_RAW = FREQ_MIN * 64'd65536 / XTAL_FREQ;
	parameter FREQ_MAX_RAW = FREQ_MAX * 64'd65536 / XTAL_FREQ;
	
	/*** VCO freq control network ***/

	reg [9:0] freq = FREQ_DEFAULT_RAW[9:0];
	reg [9:0] freq_min = FREQ_MIN_RAW[9:0];
	reg [9:0] freq_max = FREQ_MAX_RAW[9:0];
	
	parameter SLEW_CENTER = {1'b1, 9'h0};
	parameter SLEW_DIV = SLEW_RATE /* ns / Hz */
	                     * XTAL_FREQ / 64'h10000 /* ns / frequency count */
	                     / XTAL_NS /* clocks / frequency count */;
	
	reg [9:0] slew_cur = SLEW_CENTER;
	
	reg slew_slow = 0; /* i.e., become slower */
	reg slew_fast = 0; /* i.e., become faster */

	reg do_slew_fast_1a = 0;
	reg do_slew_slow_1a = 0;
	wire do_slew_fast = (slew_cur <= (SLEW_CENTER - SLEW_DIV[9:0])) & ~do_slew_fast_1a;
	wire do_slew_slow = (slew_cur >= (SLEW_CENTER + SLEW_DIV[9:0])) & ~do_slew_slow_1a;
	
	wire lockout;

	wire [9:0] freq_next = freq - {9'b0,do_slew_slow_1a} + {9'b0,do_slew_fast_1a};
	
	assign stio = {slew_slow, slew_fast};
	
	always @(posedge clk_50) begin
		if (lockout)
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
	reg [15:0] lockout_cyc_ctr = 0;
	reg last_fb = 0;
	
	wire lockout_freq_lo = lockout_cyc_ctr == CYCLES_LOCKOUT_LO[15:0];
	wire lockout_freq_hi = (fb ^ last_fb) && (lockout_cyc_ctr <= CYCLES_LOCKOUT_HI[15:0]);
	
	always @(posedge clk_50) begin
		last_fb <= fb;
		
		if (fb ^ last_fb)
			lockout_cyc_ctr <= 0;
		else if (lockout_cyc_ctr < CYCLES_LOCKOUT_LO[15:0])
			lockout_cyc_ctr <= lockout_cyc_ctr + 1;
	end
	
	wire lockout_trig = lockout_freq_lo || lockout_freq_hi;
	
	reg [15:0] lockout_tm_ctr = 0;
	always @(posedge clk_50) begin
		if (lockout_trig)
			lockout_tm_ctr <= CYCLES_LOCKOUT[15:0];
		else if (lockout_tm_ctr != 16'b0)
			lockout_tm_ctr <= lockout_tm_ctr - 1;
	end
	
	assign lockout = |lockout_tm_ctr;
	
	/*** VCO ***/
	reg [15:0] ctr = 16'h0000;
	
	wire vco; /* delayed VCO, from later. */
	
	wire vco_raw = ctr[15];
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
	wire vco_phase = ctr_ofs[15];
	
	reg [15:0] pha_ctr;
	always @(posedge clk_50) begin
		pha_ctr <= pha_ctr + 1;
		
		if (&pha_ctr) begin
			if (btns[0] & btns[1])
				shift_small <= 0;
			else if (btns[0])
				shift_small <= shift_small + 1; // phase lag
			else if (btns[1])
				shift_small <= shift_small - 1; // phase lead
		end
	end
	
	/*** Time offset logic ***/
	
	reg [7:0] shift_cyc = 8'h73;
	
	reg [128:0] vco_shift = 129'b0; /* raw VCO output, temporally delayed */
	reg [255:0] vco_phase_shift = 256'b0; /* phase shifted VCO output, temporally delayed */
	reg vco_phase_shifted = 0;
	
	always @(posedge clk_50) begin
		vco_shift <= {vco_shift[127:0], vco_raw};
		vco_phase_shift <= {vco_phase_shift[254:0], vco_phase};
		vco_phase_shifted <= vco_phase_shift[shift_cyc]; /* avoid a glitch on the output */
	end
	
	assign vco = vco_shift[128];
	assign pllout = vco_phase_shifted;
	
	reg [20:0] tm_ctr;
	always @(posedge clk_50) begin
		tm_ctr <= tm_ctr + 1;
		if (&tm_ctr) begin
			if (btns[2] & btns[3])
				shift_cyc <= 8'h80;
			else if (btns[2])
				shift_cyc <= (&shift_cyc) ? shift_cyc : (shift_cyc + 1); // time lag
			else if (btns[3])
				shift_cyc <= (~|shift_cyc) ? shift_cyc : (shift_cyc - 1); // time lead
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
	  btns[2] ? {8'h0,shift_cyc} :
	  btns[3] ? {8'h0,shift_cyc} :
	            freqlat;
	wire [3:0] display_alive = 
	   { |display[15:12],
	     |display[15:8],
	     |display[15:4],
	     1'b1
	   };
	
	wire [3:0] display_decimal = 4'b0000;
	
	Drive7Seg drive(clk_50, display, display_decimal, display_alive, cath, ano);
endmodule
