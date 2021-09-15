#!/bin/sh

st-flash --reset write bin_delivery/lynx/final/calibration-entry.bin 0x8000000
