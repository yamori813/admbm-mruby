NEWLIBDIR=newlib-3.0.0.20180831

CROSS_CFLAGS = -I../build/work/$(NEWLIBDIR)/newlib/libc/include/
CROSS_CFLAGS += -EL -G 0
CROSS_CFLAGS += -Os -g -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-strict-aliasing -fno-common -fomit-frame-pointer -G 0
CROSS_CFLAGS += -pipe -mlong-calls
CROSS_CFLAGS += -DADM5120 -DENDIAN_LITTLE=1 -DIPOLL=1 -DXPOLL=0 -march=mips32 -DCPUCFG_REGS32=1 -DCFG_INTERRUPTS=1 -DCFG_RUNFROMKSEG0=1 -DCFG_RELOC=1 -DCFE_API_ALL -DENDIAN_BIG=0

CROSS=mips
CROSSBU=mips-unknown-freebsd13.0

OBJS = exception.o bcmcore_l1cache.o exchandler.o bcmcore_ircpoll.o lib_try.o lib_queue.o lib_setjmp.o lib_physio.o cfe_timer.o bcmcore_irq.o sb_utils.o net_ether.o apientry.o bcmcore_cpuinit.o
OBJS += adm_irq.o dev_admsw_mac.o adm_cache.o

libcfe.a : $(OBJS)
	$(CROSSBU)-ar rs libcfe.a $(OBJS)

.S.o:
	$(CROSS)-gcc -O2 $(CROSS_CFLAGS) -c $<

.c.o:
	$(CROSS)-gcc -O2 $(CROSS_CFLAGS) -c $<

clean :
	rm -rf $(OBJS) libcfe.a
