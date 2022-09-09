BOARDCORE := ../../../../skyward-boardcore/
OBSW := ../../../../src/boards/

all:
	g++ -std=c++17 -O2 -o logdecoder logdecoder.cpp \
					-DCOMPILE_FOR_X86 \
					$(BOARDCORE)libs/tscpp/tscpp/stream.cpp \
	 				-I$(BOARDCORE)libs/mavlink-skyward-lib \
	 				-I$(BOARDCORE)libs/eigen \
	 				-I$(BOARDCORE)libs/tscpp \
	 				-I$(BOARDCORE)src/shared \
					-I$(OBSW)
clean:
	rm logdecoder
