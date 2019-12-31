#
# 
#

LIBDIR=../../bcm/bcmbm-mruby
NEWLIBDIR=$(LIBDIR)/newlib-3.0.0.20180831
LWIPDIR=$(LIBDIR)/lwip-2.1.2
BARESSLDIR=$(LIBDIR)/bearssl-0.6

TOOLDIR=../../bcm/hndtools-mipsel-linux-uclibc-4.2.3

CROSS=mips

CROSS_LDFLAGS = -static
CROSS_LIBS = -L./$(NEWLIBDIR)/mips/newlib
CROSS_LIBS += -L./$(TOOLDIR)/lib/gcc/mipsel-linux-uclibc/4.2.3/
CROSS_LIBS += -Lmruby/build/admtek/lib/
CROSS_LIBS += -L$(LWIPDIR)/broadcom/
CROSS_LIBS += -L./$(BARESSLDIR)/build/
CROSS_LIBS += -Lcfe/
CROSS_LIBS += -lmruby -llwip -lbearssl -lcfe -lc -lgcc

CROSS_CFLAGS = -I./$(NEWLIBDIR)/newlib/libc/include/
CROSS_CFLAGS += -Imruby/include
CROSS_CFLAGS += -I./$(LWIPDIR)/src/include -I./$(LWIPDIR)/broadcom/include
CROSS_CFLAGS += -I$(BARESSLDIR)/inc
#CROSS_CFLAGS += -Os -g -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-strict-aliasing -fno-common -fomit-frame-pointer -G 0
CROSS_CFLAGS += -pipe -mlong-calls
CROSS_CFLAGS += -DUSE_INQUEUE=1 -march=4kc

OBJS = start.o main.o syscalls.o xprintf.o adm_timer.o
OBJS += adm_ether.o mt19937ar.o net.o bear.o adm_gpio.o
OBJS += adm_switch.o
OBJS += i2c.o

RBSCRIPT = samples/hello.rb

main.bin.bz2.uboot : $(OBJS) cfe/libcfe.a
	./ver.sh
	$(CROSS)-cc $(CROSS_CFLAGS) -c ver.c
	$(CROSS)-ld $(CROSS_LDFLAGS) -T main.ld -o main.elf $(OBJS) ver.o $(CROSS_LIBS)
	$(CROSS)-objcopy -O binary main.elf main.bin
	bzip2 -f main.bin
	mkimage -A mips -O linux -T kernel -C bzip2 -a 0x80010000 -e 0x80010000 -n 'mruby VM image' -d main.bin.bz2 main.bin.bz2.uboot

image :
	./mruby/build/host/bin/mrbc -ohoge.mrb $(RBSCRIPT)
	cat main.bin.bz2.uboot hoge.mrb > main.img

start.o : start.S
	$(CROSS)-as -march=4kc -o start.o start.S

main.o : main.c
	$(CROSS)-cc $(CROSS_CFLAGS) -o main.o -c main.c

.c.o:
	$(CROSS)-cc -O2 $(CROSS_CFLAGS) -c $<

syscalls.o : syscalls.c
	$(CROSS)-cc $(CROSS_CFLAGS) -o syscalls.o -c syscalls.c

clean:
	rm -rf main.elf *.o ver.c *.uboot *.bz2
