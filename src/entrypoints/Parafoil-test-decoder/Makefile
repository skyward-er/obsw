BASE := ../../../
BOARDCORE := $(BASE)skyward-boardcore/

all:
	g++ -std=c++17 -O2 -o logdecoder Parafoil-decoder.cpp \
					$(BOARDCORE)libs/tscpp/tscpp/stream.cpp \
					-DCOMPILE_FOR_X86 \
					-I$(BOARDCORE)libs/tscpp/tscpp \
					-I$(BASE)src/boards/ParafoilTest \
					-I$(BASE)src/boards/DeathStack \
	 				-I$(BASE)src \
	 				-I$(BOARDCORE)src/shared \
	 				-I$(BOARDCORE)src/tests \
					-I$(BOARDCORE)libs \
					-I$(BOARDCORE)libs/tscpp \
					-I$(BOARDCORE)libs/eigen \
					-I$(BOARDCORE)libs/mavlink_skyward_lib \
					-I$(BOARDCORE)libs/miosix-kernel/miosix

clean:
	rm logdecoder
