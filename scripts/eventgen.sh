#!/bin/bash

DIRNAME="$(dirname $0)"
python3  $DIRNAME/../skyward-boardcore/scripts/eventgen/eventgen.py $(find $DIRNAME/../src/boards -name "*.scxml")

rm -r $DIRNAME/generated
mkdir $DIRNAME/generated
mv generated/* $DIRNAME/generated
rm -r generated

echo "The generated files are in the scripts/generated/ folder"