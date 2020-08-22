#!/bin/bash


export FLOAT_TYPE=hard

rm build/drum.elf
make

./script-all drum

