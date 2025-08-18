#
# 
#

NEWLIBDIR=build/work/newlib-3.0.0.20180831
LWIPDIR=build/work/lwip-2.1.2
BARESSLDIR=build/work/bearssl-0.6

CROSS=mips
CROSSBU=mips-unknown-freebsd13.0

CROSS_LDFLAGS = -static -EL
CROSS_LIBS = -L./$(NEWLIBDIR)/mips/el/newlib/
CROSS_LIBS += -L/usr/local/lib/gcc/mips/4.9.2/el/
CROSS_LIBS += -Lmruby/build/admtek/lib/
CROSS_LIBS += -L$(LWIPDIR)/mips4kel/
CROSS_LIBS += -L./$(BARESSLDIR)/build/
CROSS_LIBS += -Lcfe/
CROSS_LIBS += -lmruby -llwip -lbearssl -lcfe -lc -lgcc

CROSS_CFLAGS = -I./$(NEWLIBDIR)/newlib/libc/include/
CROSS_CFLAGS += -Imruby/include
CROSS_CFLAGS += -I./$(LWIPDIR)/src/include -I./$(LWIPDIR)/mips4kel/include
CROSS_CFLAGS += -I$(BARESSLDIR)/inc
CROSS_CFLAGS += -EL -G 0
#CROSS_CFLAGS += -Os -g -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-strict-aliasing -fno-common -fomit-frame-pointer -G 0
CROSS_CFLAGS += -pipe -mlong-calls
CROSS_CFLAGS += -march=4kc

OBJS = start.o main.o syscalls.o xprintf.o adm_timer.o
OBJS += adm_ether.o mt19937ar.o net.o bear.o httpsvr.o adm_gpio.o adm_uart.o
OBJS += adm_switch.o
OBJS += i2c.o

RBSCRIPT = samples/hello.rb

main.bin.bz2.uboot : $(OBJS) cfe/libcfe.a
	./ver.sh
	$(CROSS)-gcc $(CROSS_CFLAGS) -c ver.c
	$(CROSSBU)-ld $(CROSS_LDFLAGS) -T main.ld -o main.elf $(OBJS) ver.o $(CROSS_LIBS)
	$(CROSSBU)-objcopy -O binary main.elf main.bin
	bzip2 -f main.bin
	mkimage -A mips -O linux -T kernel -C bzip2 -a 0x80010000 -e 0x80010000 -n 'mruby on YABM' -d main.bin.bz2 main.bin.bz2.uboot

image :
	./mruby/build/host/bin/mrbc -ohoge.mrb $(RBSCRIPT)
	@sha256 hoge.mrb
	cat main.bin.bz2.uboot hoge.mrb > main.img

start.o : start.S
	$(CROSSBU)-as -EL -march=4kc -o start.o start.S

main.o : main.c
	$(CROSS)-gcc $(CROSS_CFLAGS) -o main.o -c main.c

.c.o:
	$(CROSS)-gcc -O2 $(CROSS_CFLAGS) -c $<

syscalls.o : syscalls.c
	$(CROSS)-gcc $(CROSS_CFLAGS) -o syscalls.o -c syscalls.c

clean:
	rm -rf main.elf *.o ver.c *.uboot *.bz2
