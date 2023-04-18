all : hello_linux

PREFIX:=../buildroot/output/host/bin/riscv32-buildroot-linux-gnu-
CC:=$(PREFIX)gcc
OBJDUMP:=$(PREFIX)objdump

CFLAGS:=-mabi=ilp32 -march=rv32imac -Os -s -g

C_S+=hello_linux.c

test : deploy
	make -C .. toolchain
	make -C ../mini-rv32ima mini-rv32ima
	../mini-rv32ima/mini-rv32ima -f ../buildroot/output/images/Image

hello_linux : hello_linux.c
	$(CC) $(CFLAGS) $(C_S) $(LDFLAGS) -o $@ 
	$(OBJDUMP) -S $@.gdb > $@.S

deploy : hello_linux
	cp $^ ../buildroot/output/target/root

rebuild :
	make -C.. toolchain 

clean :
	rm -rf hello_linux hello_linux.gdb hello_linux.S
