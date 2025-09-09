BOARDCORE := ../../skyward-boardcore/
OBSW := ../../src/

all:
	g++ -std=c++17 -o eventdumper EventDumper.cpp \
					-DCOMPILE_FOR_X86 \
					-DCOMPILE_FOR_HOST \
					-I$(BOARDCORE)libs/miosix-host \
	 				-I$(BOARDCORE)libs/mavlink-skyward-lib \
	 				-I$(BOARDCORE)libs/eigen \
					-I$(BOARDCORE)libs/socrate/include \
	 				-I$(BOARDCORE)src/shared \
					-I$(OBSW)
clean:
	rm eventdumper
