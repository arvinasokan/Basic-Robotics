#!/usr/bin/env python
import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)
#float lap = 0.7/1000
LHP=0
RHP=0
RKN=0
RAP=0
LKN=0
LAP=0
RHY=0

def init(REB,LEB):
  for i in range (0,100):    
    REB=REB+0.016
    ref.ref[ha.REB] = -REB
    LEB=LEB+.016
    ref.ref[ha.LEB] = -LEB
    r.put(ref)
    time.sleep(.05)
def crouchleft(LHP,LKN,LAP):
  for i in range (0,100):
    LHP=LHP+.0025
    ref.ref[ha.LHP] = -LHP 
    LKN=LKN+.0051
    ref.ref[ha.LKN] =  LKN
    LAP = LAP +0.0025
    ref.ref[ha.LAP] = -LAP
    r.put(ref)
    time.sleep(.05)
  time.sleep(1)
  for i in range (0,100):
    LHP=LHP-.0025
    ref.ref[ha.LHP] = -LHP 
    LKN=LKN-.0051
    ref.ref[ha.LKN] =  LKN
    LAP = LAP -0.0025
    ref.ref[ha.LAP] = -LAP
    r.put(ref)
    time.sleep(.05)   
def crouchright(RHP,RKN,RAP):
  for i in range (0,100):
    RHP=RHP+.0025
    ref.ref[ha.RHP] = -RHP 
    RKN=RKN+.0051
    ref.ref[ha.RKN] =  RKN
    RAP = RAP +0.0025
    ref.ref[ha.RAP] = -RAP
    r.put(ref)
    time.sleep(0.05)
  time.sleep(1)
  for i in range (0,100):
    RHP=RHP-.0025
    ref.ref[ha.RHP] = -RHP 
    RKN=RKN-.0051
    ref.ref[ha.RKN] =  RKN
    RAP = RAP -0.0025
    ref.ref[ha.RAP] = -RAP
    r.put(ref)
    time.sleep(.05) 
def moveleft(RHR,LHR,RAR,LAR):
  for i in range (0,100):  
    RHR=RHR+0.0015
    ref.ref[ha.RHR] =-RHR 
    LHR=LHR+0.0015
    ref.ref[ha.LHR] =-LHR
    RAR=RAR+.0016
    ref.ref[ha.RAR] =RAR
    LAR=LAR+.0016
    ref.ref[ha.LAR] =LAR 
    r.put(ref)
    time.sleep(.05)
def moveright(RHR,LHR,RAR,LAR):
  for i in range (0,100):  
    RHR=RHR+0.0015
    ref.ref[ha.RHR] =RHR 
    LHR=LHR+0.0015
    ref.ref[ha.LHR] =LHR
    RAR=RAR+.0015
    ref.ref[ha.RAR] =-RAR
    LAR=LAR+.0015
    ref.ref[ha.LAR] =-LAR
    r.put(ref)
    time.sleep(.05)
def rightstep(RHP,RKN,RAP):
  for i in range(0,100):  
    RHP=RHP+.014
    ref.ref[ha.RHP] = -RHP
    RKN=RKN+0.0196884
    ref.ref[ha.RKN] =  RKN
    RAP=RAP+.0048
    ref.ref[ha.RAP] = -RAP
    r.put(ref)
    time.sleep(.1)
def rightreturn(RHP,RKN,RAP):
  
    ref.ref[ha.RHP] = 0
    
    ref.ref[ha.RKN] =  0
    
    ref.ref[ha.RAP] =  0
    r.put(ref)
    time.sleep(.1)
def leftstep(LHP,LKN,LAP):
  for i in range(0,100):  
    LHP=LHP+.014
    ref.ref[ha.LHP] = -LHP
    LKN=LKN+0.0196884
    ref.ref[ha.LKN] =  LKN
    LAP=LAP+.0048
    ref.ref[ha.LAP] = -LAP
    r.put(ref)
    time.sleep(.1)
init(0,0)
time.sleep(4)
moveleft(0,0,0,0)
rightstep(0,0,0)
time.sleep(3)
for i in range(0,5):
  crouchleft(0,0,0)
rightreturn(0,0,0)
'''time.sleep(2)
moveright(0,0,0,0)
leftstep(0,0,0)
time.sleep(3)
for i in range(0,5):
  crouchright(0,0,0)

'''
r.close()
s.close()
