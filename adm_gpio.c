/*
 * Copyright (c) 2019 Hiroki Mori. All rights reserved.
 *
 * Todo: LED GPIO support
 */

#define	CODE		0xb2000000
#define	GPIO_CONF0	0xb20000b8
#define	PORT_LED	0xb2000100

#define	MODULE_ADM5120	7
#define	MODULE_ADM5120P	8

unsigned long gpio_getctl()
{

	return 0;
}

void gpio_setctl(unsigned long val)
{

}

void gpio_setled(int port, unsigned long val)
{
unsigned long *lptr;

	lptr = (unsigned long *)PORT_LED + port;

	*lptr = val;
}

unsigned long gpio_getdir()
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	return (*lptr >> 16) & 0xff;
}

void gpio_setdir(unsigned long val)
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	*lptr = (*lptr & ~0xff000000) | (val << 16);
}

unsigned long gpio_getdat()
{
unsigned long *lptr;
int led;
int i;

	led = 0;
	for (i = 0; i < 5; ++i) {
		lptr = (unsigned long *)PORT_LED + i;
		led |= ((*lptr >> 12) & 0x7) << (i * 3);
	}

	lptr = (unsigned long *)GPIO_CONF0;

	return ((*lptr >> 8) & 0xff) | (led << 16);
}

void gpio_setdat(unsigned long val)
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	*lptr = (*lptr & 0x000f0000) | (val << 24);
}

int getarch()
{
unsigned long *lptr;

	lptr = (unsigned long *)CODE;

	if ((*lptr >> 29) & 1 == 1)
		return MODULE_ADM5120P;
	else
		return MODULE_ADM5120;
}
