#include <mruby.h>
#include <mruby/string.h>
#include <mruby/irep.h>

#include "xprintf.h"

extern char version[];

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

#ifdef CAHCHE_TEST
void cache_test()
{
	*(volatile unsigned int *)0x80700000 = 0x1234;
	xprintf("MORIMORI %x\n", *(volatile unsigned int *) 0x80700000);
	*(volatile unsigned int *)0xa0700000 = 0xbeef;
//	dcache_wback_inv(0x80700000, 32);
//	flush_cache(0x80700000, 32);
}
#endif
 
int main(void)
{
unsigned char *sizep;
int vmsize;
unsigned char *mrbp;
int mrbsize;
unsigned char *mrbbuf;
int bootsize;
unsigned char hash[32];
int i;

	bcmcore_cpuinit();

	xfunc_out=put;

	print(version);

#ifdef CAHCHE_TEST
	cache_test();
	xprintf("MORIMORI %x\n", *(volatile unsigned int *) 0x80700000);
#endif

	mt19937ar_init();

	cfe_setup_exceptions();
	cfe_irq_init();

	timer_init();

	adm_irq_init();

	bootsize = 0x40000;

	sizep = 0xbfc00000 + bootsize + 0xc;
	vmsize = *sizep << 24 | *(sizep + 1) << 16 |
	    *(sizep + 2) << 8 | *(sizep + 3);
	vmsize += 64;
	mrbp = 0xbfc00000 + bootsize + vmsize;
	if (*(mrbp + 0) == 0x52 && *(mrbp + 1) == 0x49 &&
	    *(mrbp + 2) == 0x54 && *(mrbp + 3) == 0x45) {
		mrbsize = *(mrbp + 0x8) << 24 | *(mrbp + 0x9) << 16 |
		    *(mrbp + 0xa) << 8 | *(mrbp + 0xb);
		xprintf("MRB SIZE %d\n", mrbsize);
		mrbbuf = malloc(mrbsize);
		memcpy(mrbbuf, mrbp, mrbsize);
		mksha256(mrbbuf, mrbsize, hash);
		xprintf("MRB SHA256 ");
		for (i = 0; i < 32; ++i)
			xprintf("%02x", hash[i]);
		xprintf("\n");

		mrb_state *mrb;
		mrb = mrb_open();
		mrb_load_irep( mrb, mrbbuf);
		if (mrb->exc) {
			mrb_value exc = mrb_obj_value(mrb->exc);
			mrb_value inspect = mrb_inspect(mrb, exc);
			print(mrb_str_to_cstr(mrb, inspect));
		}
		mrb_close(mrb);
	} else {
		print("can't find mrb code on flash\n");
	}

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
