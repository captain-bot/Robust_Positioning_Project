#!/bin/bash

g++ -std=c++11 -DNDEBUG -Wall -Wstrict-prototypes -fPIC -I/home/nobug-ros/anaconda3/include/python3.6m -c myIKFastWrapRightArm.cpp -o ikModuleRight.o -llapack

g++ -shared ikModuleRight.o -o ikModuleRight.so -llapack

# python installer.py
