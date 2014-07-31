#!/usr/bin/env python
import ach
import refer
import time
import math 
re=refer
ref =re.my_msg()
r = ach.Channel('my_channel')
r.flush()
def IK(x,y):
    l1=4000
    l2=4000
    l3=1000
    phi=0;
    xw=x-l3*math.cos(0)
    yw=y-l3*math.sin(0)
    if(xw==0 and yw ==0):
      theta1=0
      theta2=0
      theta3=0
    else:
      theta2=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
      theta1=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
      theta3=phi-theta2-theta1;
    return theta1,theta2,theta3
ref.ref[0]=0
ref.ref[1]=0
ref.ref[2]=0
ref.ref[3]=0
ref.ref[4]=0
ref.ref[5]=0
ref.ref[6]=0
ref.ref[7]=0
ref.ref[8]=0
r.put(ref)
x=0.001
time.sleep(1)
while True:
  (x,y,z)=IK(7000,0)
  x=x/1000
  y=y/1000
  z=z/1000
  a=0
  b=0
  c=0
  for i in range(0,1000):
    a=a+x
    b=b+y
    c=c+z
    ref.ref[0]=a
    ref.ref[1]=0
    ref.ref[2]=0
    ref.ref[3]=0
    ref.ref[4]=b
    ref.ref[5]=0
    ref.ref[6]=0
    ref.ref[7]=0
    ref.ref[8]=c
    time.sleep(.001) 
    r.put(ref)
  for i in range(0,1000):
    a=a-x
    b=b-y
    c=c-z  
    ref.ref[0]=a
    ref.ref[1]=0
    ref.ref[2]=0
    ref.ref[3]=0
    ref.ref[4]=b
    ref.ref[5]=0
    ref.ref[6]=0
    ref.ref[7]=0
    ref.ref[8]=c
    time.sleep(.001) 
    r.put(ref) 
r.close()
