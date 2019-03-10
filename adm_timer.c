#include "cfe/cfe_irq.h"

#include "time.h"

#define Watchdog0	0xb20000c0

unsigned int alarm;

unsigned int interval;
unsigned long starttime;
static volatile unsigned int jiffies = 0;

static void
timer_isr(void *arg)
{
	net_poll();

	alarm += interval;
	_setalarm(alarm);
	++jiffies;
}

timer_init()
{
int clk;

	clk = adm_timer_init();
	cfe_timer_init(clk);
	xprintf("Clock : %d\n", clk);
	interval = (clk / 2) / 100;

	cfe_request_irq(5, timer_isr, 0, CFE_IRQ_FLAGS_SHARED, 0);
	cfe_enable_irq(5);
	alarm = _getticks() + interval;
	_setalarm(alarm);
}

void watchdog_start(int sec)
{
unsigned long *lptr;

	lptr = (unsigned long *)Watchdog0;

	*lptr = (1 << 31) | ((sec * 100) << 16);
}

void watchdog_reset()
{
unsigned long *lptr;
unsigned long reg;

	lptr = (unsigned long *)Watchdog0;

	reg = *lptr;

	*lptr = reg & 0xffff0000;
}

void watchdog_stop()
{
unsigned long *lptr;

	lptr = (unsigned long *)Watchdog0;

	*lptr = 0;
}

int sys_now()
{
	return jiffies * 10;
}

void reset_counter()
{
	jiffies = 0;
}

delay_ms(int ms)
{
	cfe_usleep(ms * 1000);
}

time_t
time(time_t *t)
{
	return sys_now()/1000 + starttime;
}

udelay(int us)
{
        cfe_usleep(us);
}                                                                              

