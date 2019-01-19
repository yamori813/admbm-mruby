#include <mruby.h>
#include <mruby/string.h>
#include <mruby/irep.h>

#include "xprintf.h"

#include "hoge.c"

void put(char c)
{
	volatile char* lsr = (volatile char*)0xb2600018; // Line status register.
	volatile char* thr = (volatile char*)0xb2600000; // Transmitter holding register.
 
	while(((*lsr) & 0x20) != 0) ; // Wait until THR is empty.
 
	*thr = c;
}

void print(char *ptr)
{
	while(*ptr) {
		put(*ptr);
		++ptr;
	}
}
 
int main(void)
{

	xfunc_out=put;

	cfe_setup_exceptions();
	cfe_irq_init();

	/*Clear relevant SR bits*/
	_exc_clear_sr_exl();
	_exc_clear_sr_erl();

	timer_init();

	adm_irq_init();

        mrb_state *mrb;
        mrb = mrb_open();
        mrb_load_irep( mrb, bytecode);
        mrb_close(mrb);

	return 0;
}

int sr;

cli()
{
        sr = cfe_irq_disable();
}

sti()
{
        cfe_irq_enable(sr);
}

int getarch()
{
	return 0;
}
