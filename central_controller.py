#!/usr/bin/env python
import sys
import cv2
import numpy as np
from turntable import Turntable, clamp_radians
import time

"""Globals"""
# control period dT


# For turntable


class Central_controller():
    def __init__(self):
        self.dT = 1/120.0
        self.turntables = []
        self.turntables.append(Turntable(1, self.dT))
        self.turntables.append(Turntable(2, self.dT))
        

    def run(self):
    	time.sleep(1)
        while(True):
            start = time.time()
            """ Code starts"""
            for i in range(2):
                self.turntables[i].update()


            """ Code ends"""
            elapse = time.time() - start
            if elapse < self.dT:
                time.sleep(self.dT - elapse)
            else:
                print "NO!!! To much computation time each loop!!!"

            # break

"""Camera call back"""


"""main function"""
def main():
    ctl = Central_controller()
    ctl.run()
    

if __name__ == '__main__':
    main()