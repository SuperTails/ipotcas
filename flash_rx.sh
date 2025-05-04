#!/bin/bash

openocd -f interface/stlink-v2-1.cfg -f target/stm32f7x.cfg -c "adapter serial 066AFF575487884867084720; init; reset halt; flash write_image erase build/ipotcas2.elf; reset; exit"