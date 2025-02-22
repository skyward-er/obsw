BOARDCORE := ../../../skyward-boardcore/
OBSW := ../../../src/

all:
	g++ -std=c++20 -O2 -o logdecoder logdecoder.cpp \
					-DCOMPILE_FOR_X86 \
					-DCOMPILE_FOR_HOST \
					-DFMT_HEADER_ONLY \
					$(BOARDCORE)libs/tscpp/tscpp/stream.cpp \
					-I$(BOARDCORE)libs/miosix-host \
	 				-I$(BOARDCORE)libs/mavlink-skyward-lib \
	 				-I$(BOARDCORE)libs/fmt/include \
	 				-I$(BOARDCORE)libs/eigen \
	 				-I$(BOARDCORE)libs/tscpp \
	 				-I$(BOARDCORE)src/shared \
					-I$(OBSW)
clean:
	rm logdecoder
