# -*- coding: utf-8 -*-
"""
Created on Tue Jul 11 13:53:25 2023

@author: ryan.robinson
"""

import stepper as stpm
import servo as serm
import time


ROW3 = 479830
ROW2 = 220097
ROW1 = -39636
ROW0 = -307381

COL6 = 124008
COL5 = 101590
COL4 = 79170
COL3 = 56750
COL2 = 34340
COL1 = 11900
COL0 = -10500


try:
    sm = stpm.Stepper_Motor()
    sm.move_abs(COL0)
    time.sleep(1)
    sm.move_abs(COL1)
    time.sleep(1)
    sm.move_abs(COL2)
    time.sleep(1)
    sm.move_abs(COL3)
    time.sleep(1)
    sm.move_abs(COL4)
    time.sleep(1)
    sm.move_abs(COL5)
    time.sleep(1)
    sm.move_abs(COL6)
    time.sleep(1)
    a = sm.get_pos()
    print(a)
    # sm.home()

finally:
    sm.close()
