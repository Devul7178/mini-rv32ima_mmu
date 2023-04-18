EXE:=duktapetest

all : $(EXE)

CFLAGS?=-march=rv32ima -mabi=ilp32 -fPIE -pie -Os -s -static
LDFLAGS?=-Wl,-r -Wl,-elf2flt --entry main,-s -Wl,-z,max-page-size=4096

DUKTAPEVER:=duktape-2.7.0
DUKTAPEBLD:=src-noline
CFLAGS+=-I$(DUKTAPEVER)/$(DUKTAPEBLD)

CC:=../../buildroot/output/host/bin/riscv32-buildroot-linux-gnu-gcc
OBJDUMP:=../../buildroot/output/host/bin/riscv32-buildroot-linux-gnu-objdump

C_S+=$(DUKTAPEVER)/$(DUKTAPEBLD)/duktape.c
C_S+=$(DUKTAPEVER)/examples/cmdline/duk_cmdline.c
C_S+=$(DUKTAPEVER)/extras/console/duk_console.c
CFLAGS+=-DDUK_CMDLINE_CONSOLE_SUPPORT -I$(DUKTAPEVER)/extras/console

$(EXE) : $(DUKTAPEVER)
	$(CC) $(C_S) $(CFLAGS) $(LDFLAGS) -o $@
	$(OBJDUMP) -S $@ > $@.S

deploy : $(EXE) fizzbuzz.js
	cp $^ ../../buildroot/output/target/root

rebuild : deploy
	make -C../.. toolchain 

# sudo apt-get install node node-js-yaml node-yamlish python-yaml

$(DUKTAPEVER) :
	wget https://duktape.org/$(DUKTAPEVER).tar.xz -O $(DUKTAPEVER).tar.xz
	tar xvpf $(DUKTAPEVER).tar.xz

clean :
	rm -rf $(EXE) $(EXE).gdb $(EXE).S


