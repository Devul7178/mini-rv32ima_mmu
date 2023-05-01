all : mini-rv32ima mini-rv32ima.flt

mini-rv32ima : mini-rv32ima.c mini-rv32ima.h default64mbdtc2.h
	# for debug
	gcc -o $@ $< -g -O0 -Wall
	gcc -o $@.tiny $< -Os -ffunction-sections -fdata-sections -Wl,--gc-sections -fwhole-program -s

mini-rv32ima.flt : mini-rv32ima.c mini-rv32ima.h
	../buildroot/output/host/bin/riscv32-buildroot-linux-gnu-gcc -O4 -funroll-loops -s -march=rv32ima -mabi=ilp32 -fPIC $< -o $@

# Deply with:  make clean all && cp mini-rv32ima.flt ../buildroot/output/target/root/ && make -C .. toolchain && make testkern

profile : Image.ProfileTest
	echo "#!/bin/sh" > ../buildroot/output/target/etc/init.d/SLocal
	echo 'if [ "$$1" = "start" ]; then /root/mini-rv32ima.flt -f /root/Image.ProfileTest -plt 4 -m 0xfff000; poweroff; fi' >> ../buildroot/output/target/etc/init.d/SLocal
	chmod +x ../buildroot/output/target/etc/init.d/SLocal
	cp Image.ProfileTest ../buildroot/output/target/root/
	make clean all && cp mini-rv32ima.flt ../buildroot/output/target/root/ && make -C .. toolchain && ./mini-rv32ima -f ../buildroot/output/images/Image -plt 4 -m 0x6000000
	echo "Be sure to record the immediately above value."

Image.ProfileTest :
	wget https://github.com/cnlohr/mini-rv32ima-images/raw/master/images/Image.ProfileTest-linux-5.18.0-rv32nommu.zip
	unzip Image.ProfileTest-linux-5.18.0-rv32nommu.zip

Image-emdoom-MAX_ORDER_14 :
	wget https://github.com/cnlohr/mini-rv32ima-images/raw/master/images/Image-emdoom-MAX_ORDER_14.zip
	unzip Image-emdoom-MAX_ORDER_14.zip

testdoom : Image-emdoom-MAX_ORDER_14 mini-rv32ima
	./mini-rv32ima -f Image-emdoom-MAX_ORDER_14 -m 0x3000000 -k "console=ttyS0"

testkern : mini-rv32ima
	./mini-rv32ima -f Image -m 0x6000000 -k "console=hvc0"

testbare : mini-rv32ima
	./mini-rv32ima -f ../baremetal/baremetal.bin

DownloadedImage :
	wget https://github.com/cnlohr/mini-rv32ima-images/raw/master/images/linux-6.1.14-rv32nommu-cnl-1.zip -O linux-6.1.14-rv32nommu-cnl-1.zip
	unzip linux-6.1.14-rv32nommu-cnl-1.zip
	mv Image DownloadedImage

testdlimage : mini-rv32ima
	cp ../buildroot/output/images/Image ./
	cp ../buildroot/output/images/fw_payload.bin ./
	./mini-rv32ima -f Image -x fw_payload.bin

# For dumping specific binaries and info.
# SBI is not currently working
#testsbi : mini-rv32ima
#	./mini-rv32ima -f ../opensbi/this_opensbi/platform/riscv_emufun/firmware/fw_payload.bin
#dumpsbi : 
#	../buildroot/output/host/bin/riscv32-buildroot-linux-uclibc-objdump -S ../opensbi/this_opensbi/platform/riscv_emufun/firmware/fw_payload.elf >fw_payload.S

# For converting the .dtb into a .h file for embeddding.
bintoh :
	# echo "#include <stdio.h>" > bintoh.c
	# echo "int main(int argc,char ** argv) {if(argc==1) return -1; int c, p=0; printf( \"static const unsigned char %s[] = {\", argv[1] ); while( ( c = getchar() ) != EOF ) printf( \"0x%02x,%c\", c, (((p++)&15)==15)?10:' '); printf( \"};\" ); return 0; }" >> bintoh.c
	rm bintoh
	gcc bintoh.c -o bintoh

default64mbdtc2.h : bintoh	
	# rm default64mbdtc2.h		
	# rm sixtyfourmb2.dtb
	# make sixtyfourmb2.dtb
	./bintoh sixtyfourmb2.dtb > default64mbdtc2.h
	# ./bintoh sixtyfourmb2 < $< > $@
	# ./bintoh default64mbdtb < $< > $@
	# ./bintoh virt < $< > $@
	# ./bintoh sixtyfourmb2 < $< > $@
	# WARNING: sixtyfourmb.dtb MUST hvave at least 16 bytes of buffer room AND be 16-byte aligned.
	#  dtc -I dts -O dtb -o sixtyfourmb.dtb sixtyfourmb.dts -S 1536

sixtyfourmb2.dtb :
	# rm sixtyfourmb2.dtb
	# dtc -I dts -O virt -o $@ $^	
	dtc -I dts -O dtb -f sixtyfourmb2.dts -o sixtyfourmb2.dtb
	rm ../buildroot/output/images/sixtyfourmb2.dtb
	cp sixtyfourmb2.dtb ../buildroot/output/images

dumpkern :
	../buildroot/output/host/bin/riscv32-buildroot-linux-uclibc-objdump -S ../buildroot/output/build/linux-5.18/vmlinux >fw_payload.S
	../buildroot/output/host/bin/riscv32-buildroot-linux-uclibc-objdump -t ../buildroot/output/build/linux-5.18/vmlinux >fw_payload.t

clean :
	rm -rf mini-rv32ima mini-rv32ima.flt

