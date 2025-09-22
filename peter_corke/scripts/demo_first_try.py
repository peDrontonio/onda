#!/usr/bin/env python3
import numpy as np
from first_try import RRRPRManipulator

        
def main():
    robot = RRRPRManipulator(l1=10.0, l2=0.5, l3=0.5, l4=10.0, l5=2.0)
    robot.dynamic_display()
  
if __name__ == '__main__':
    main()