BASE := ../../../
BOARDCORE := $(BASE)skyward-boardcore/

all:
	g++ -std=c++17 -O2 -o logdecoder windtunnel-decoder.cpp \
					-DCOMPILE_FOR_X86 \
					$(BOARDCORE)libs/tscpp/stream.cpp \
					-I$(BASE)src/boards/DeathStack \
	 				-I$(BOARDCORE)src/shared \
	 				-I$(BOARDCORE)src/tests \
					-I$(BOARDCORE)libs \
					-I$(BOARDCORE)libs/miosix-kernel/miosix

clean:
	rm logdecoder
