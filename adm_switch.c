/*
 * Copyright (c) 2019 Hiroki Mori. All rights reserved.
 *
 */

#define	PHY_cntl1	0xb2000068
#define	PHY_cntl2	0xb200006c

#define	RC		(1 << 14)
#define	WC		(1 << 13)
#define	ROR		2
#define WOD		1

int readmdio(unsigned int addr, unsigned int reg, unsigned int *dat)
{
unsigned long *lptr;
int count;

	lptr = (unsigned long *)PHY_cntl1;

	*lptr = RC | (reg << 8) | addr;

	lptr = (unsigned long *)PHY_cntl2;

	count = 0;
	while (*lptr & ROR == 0) {
		udelay(1);
		++count;
		if (count == 100)
			return 0;
	}

	*dat = *lptr >> 16;

	return 1;
}
