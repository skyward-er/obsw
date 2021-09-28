#!/bin/sh

st-flash --reset write bin_delivery/lynx/final/ramtest.bin 0x8000000
