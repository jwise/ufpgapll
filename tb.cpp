#include <stdlib.h>

#include "verilated.h"

#include "Vufpgapll.h"
#if VM_TRACE
#include <verilated_vcd_c.h>  
VerilatedVcdC* tfp;
#endif

#if VM_TRACE
void _close_trace() {
	if (tfp) tfp->close();
}
#endif

int main(int argc, char **argv, char **env) {
	Vufpgapll *tb = new Vufpgapll;
	int time = 0;
	
#if VM_TRACE
	Verilated::traceEverOn(true);
	tfp = new VerilatedVcdC;
	tb->trace(tfp, 99);
	tfp->open("trace.vcd");
	atexit(_close_trace);
#endif

	tb->btns = 0;
	tb->sw = 0;
	tb->xtal = 0;
	tb->fb_u = 0;

#define XTAL 50000000

#if VM_TRACE
#define TRACE tfp->dump((1e9 / (XTAL * 2)) * time)
#else
#define TRACE
#endif

#define FREQ 180000
#define CYC (XTAL / (FREQ * 2))

	while (time < 1000000) {
		tb->xtal = 0;

		tb->eval();
		TRACE;
		time++;
		
		tb->xtal = 1;
		if (((time / 2) % CYC) == 0)
			tb->fb_u = !tb->fb_u;
		tb->eval();
		TRACE;
		time++;
	}
	
	return 0;
}
