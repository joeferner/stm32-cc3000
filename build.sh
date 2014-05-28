#!/bin/bash -e

mkdir -p build
cd build
rm stm32-cc3000.elf || echo "Cannot remove. stm32-cc3000.elf not build?"
make stm32-cc3000.bin && \
make stm32-cc3000.list
