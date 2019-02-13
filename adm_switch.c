/*
 * Copyright (c) 2019 Hiroki Mori. All rights reserved.
 *
 */

#define	PHY_St		0xb2000014
#define	PHY_cntl0	0xb2000068
#define	PHY_cntl1	0xb200006c

#define	RC		(1 << 14)
#define	WC		(1 << 13)
#define	ROR		2
#define WOD		1

unsigned long physt()
{
unsigned long *lptr;

	lptr = (unsigned long *)PHY_St;

	return *lptr;
}

/* This is only valid mdio for GMII. Not work for Switch PHY. */

int readmdio(unsigned int addr, unsigned int reg, unsigned int *dat)
{
unsigned long *lptr;
unsigned long cntl1;
int count;

	lptr = (unsigned long *)PHY_cntl0;

	*lptr = RC | (reg << 8) | addr;

	lptr = (unsigned long *)PHY_cntl1;

	count = 0;
	cntl1 = *lptr;
	while ((cntl1 & ROR) != ROR) {
		delay_ms(1);
		++count;
		if (count == 100)
			return 0;
		cntl1 = *lptr;
	}

	*dat = cntl1 >> 16;

	return 1;
}
