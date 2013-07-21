#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Steinhart–Hart coefficients

import math

# input data
R25 = 100000

R1 = 3.324*R25
R2 = 0.016419*R25
R3 = 0.0010716*R25

T1 = 0   + 273.15
T2 = 150 + 273.15
T3 = 300 + 273.15

# variable change
Y1 = 1/T1
Y2 = 1/T2
Y3 = 1/T3

L1 = math.log(R1)
L2 = math.log(R2)
L3 = math.log(R3)

# Gauss pivot and solving
Gamma2 = (Y2-Y1) / (L2 - L1)
Gamma3 = (Y3-Y1) / (L3 - L1)

C = ((Gamma3-Gamma2)/(L3-L2))*1/(L1+L2+L3)
B = Gamma2 - C * (math.pow(L1,2) + L1*L2 + math.pow(L2,2))
A = Y1 - (B + math.pow(L1,2) * C) * L1



#A = 0.0010891529057991591
#B = 0.00021669173957870983
#C = 8.913929055831574e-08
R=R1
T = 1/ (A + B * math.log(R) + C * math.pow(math.log(R),3) )
print("verif T1 : " + str(T - T1))

R=R2
T = 1/ (A + B * math.log(R) + C * math.pow(math.log(R),3) )
print("verif T2 : " + str(T - T2))

R=R3
T = 1/ (A + B * math.log(R) + C * math.pow(math.log(R),3) )
print("verif T3 : " + str(T -T3))

R=100000
T = 1/ (A + B * math.log(R) + C * math.pow(math.log(R),3) )
print("verif 25°C : " + str(T-273.15))

print("A=" + str(A))
print("B=" + str(B))
print("C=" + str(C))
