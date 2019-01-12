#
# 
#

NEWLIBDIR=../../bcm/bcmbm-mruby/newlib-3.0.0.20180831
TOOLDIR=../../bcm/hndtools-mipsel-linux-uclibc-4.2.3

CROSS=mips

CROSS_LDFLAGS = -static
CROSS_LIBS = -L./$(NEWLIBDIR)/mips/newlib
CROSS_LIBS += -L./$(TOOLDIR)/lib/gcc/mipsel-linux-uclibc/4.2.3/
CROSS_LIBS += -Lmruby/build/admtek/lib/
CROSS_LIBS += -lmruby -lc -lgcc

CROSS_CFLAGS = -I./$(NEWLIBDIR)/newlib/libc/include/
CROSS_CFLAGS += -Imruby/include
CROSS_CFLAGS += -Os -g -fno-pic -mno-abicalls
CROSS_CFLAGS += -fno-strict-aliasing -fno-common -fomit-frame-pointer -G 0
CROSS_CFLAGS += -pipe -mlong-calls

OBJS = start.o main.o syscalls.o

main.elf : $(OBJS)
	$(CROSS)-ld $(CROSS_LDFLAGS) -T main.ld -o main.elf $(OBJS) $(CROSS_LIBS)

start.o : start.S
	$(CROSS)-as -o start.o start.S

main.o : main.c
	../../bcm/bcmbm-mruby/mruby/build/host/bin/mrbc -Bbytecode hoge.rb
	$(CROSS)-cc $(CROSS_CFLAGS) -o main.o -c main.c

syscalls.o : syscalls.c
	$(CROSS)-cc $(CROSS_CFLAGS) -o syscalls.o -c syscalls.c

clean:
	rm -rf main.elf *.o hoge.c
