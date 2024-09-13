/*
 *
 */

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
