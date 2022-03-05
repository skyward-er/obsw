# RAM Test

This entrypoint is useful to test the external RAM of a board.

It writes into the external RAM and then reads back the content to check its correctness and also the memory retention capability.

### Usage

* In `ramtest.cpp` :
    - Set the `ramBase` value to the starting address of the external memory, as defined in the linker script (e.g. `0xd0000000`).
    - Set the `ramSize` value to the size of the external memory in bytes, as defined in the linker script (e.g. for 8 MB : `8*1024*1024`).
* `__ENABLE_XRAM` must be defined, in Miosix's `config/Makefile.inc` (in the section corresponding to your board) or in `sbs.conf` (in the `ramtest` entrypoint section).
* In `miosix-kernel/miosix/config/Makefile.inc`, in the section of your board, select the correct linker script. _The correct linker is not the one that defines the external RAM_. **You must select (uncomment) the linker script for your board that ends with "rom"**. This is a must, because the linker script could place some executable sections (such as the `bss`) in the external ram : the `ramtest` will then crash when trying to write to those same addresses.
