#KERN_SRC=../../linux-xlnx

obj-m += isl22313.o

all:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules

clean:
	rm -f *.o *.ko modules.order Module.symvers *mod.c

