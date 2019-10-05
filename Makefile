all:
	@echo "MAKEFILE WRAPPER. PLEASE USE ./sbs FOR A BETTER EXPERIENCE :)"
	@python3 skyward-boardcore/sbs -v

.PHONY: clean

clean:
	@echo "MAKEFILE WRAPPER. PLEASE USE ./sbs FOR A BETTER EXPERIENCE :)"
	@python3 skyward-boardcore/sbs -c
