#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: shared_memory_index.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 1.0
Description: 
    Indexes for raspberry pi data for easier access

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: os, sys
"""

from multiprocessing import shared_memory
import numpy as np
import logging

logger = logging.getLogger(__name__)
# Debug flag (can be set externally or toggled)

logger.info("shared_memory_index.py")


# GPIO indexes
ISOLATED_INPUT_1 = 0
ISOLATED_INPUT_2 = 1
ISOLATED_INPUT_3 = 2

ISOLATED_OUTPUT_1 = 3  
ISOLATED_OUTPUT_2 = 4
ISOLATED_OUTPUT_3 = 5

ESTOP_2 = 6    
ESTOP_1 = 7    
# https://forums.raspberrypi.com/viewtopic.php?t=128032
# https://stackoverflow.com/questions/24226310/valueerror-the-channel-sent-is-invalid-on-a-raspberry-pi-controlling-gpio-pin
#STOP_MCU =  # BOARD 27 ->  RESERVED PIN DOUBLECHECK FOR MODEL

GPIO_1 = 8
GPIO_2 = 9
GPIO_3 = 10
GPIO_4 = 11

LED1 = 12
LED2 = 13
BUTTON2 = 14
BUTTON1 = 15

