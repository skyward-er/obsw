BASE := ../../../
BOARDCORE := $(BASE)skyward-boardcore/

all:
	g++ -std=c++17 -O2 -o logdecoder death-stack-x-decoder.cpp \
					-DCOMPILE_FOR_X86 \
					$(BOARDCORE)libs/tscpp/stream.cpp \
					-I$(BOARDCORE)src/shared/drivers/Xbee \
					-I$(BOARDCORE)src/shared/logger \
					-I$(BASE)src/boards/DeathStack \
					-I$(BASE)src/boards/DeathStack/NavigationSystem \
	 				-I$(BOARDCORE)src/shared \
	 				-I$(BOARDCORE)src/tests \
					-I$(BOARDCORE)libs \
					-I$(BOARDCORE)libs/miosix-kernel/miosix \
					-I$(BOARDCORE)libs/eigen \
					-I$(BOARDCORE)src/shared/sensors/analog/battery \
					-I$(BOARDCORE)src/shared/sensors/analog/current

clean:
	rm logdecoder
