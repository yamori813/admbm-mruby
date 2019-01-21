/*
 * Copyright (c) 2019 Hiroki Mori. All rights reserved.
 *
 * Todo: LED GPIO support
 */

#define	GPIO_CONF0	0xb20000b8

unsigned long gpio_getctl()
{

	return 0;
}

void gpio_setctl(unsigned long val)
{

}

unsigned long gpio_getdir()
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	return (*lptr >> 16) & 0xf;
}

void gpio_setdir(unsigned long val)
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	*lptr = val << 16;
}

unsigned long gpio_getdat()
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	return (*lptr >> 8) & 0xf;
}

void gpio_setdat(unsigned long val)
{
unsigned long *lptr;

	lptr = (unsigned long *)GPIO_CONF0;

	*lptr = (*lptr & 0x0f0f00) | (val << 24);
}
