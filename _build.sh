#!/bin/bash

cp .config.$1 .config
mkdir -p outfw
make clean && make && mv out/klipper.bin outfw/${1}_klipper.bin
