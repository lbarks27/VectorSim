import math
import numpy as np

def rungeKutta4(f, t, y, h): #returns kwmean + h, must use v(n+1) = v(n) + kwmean +h
    k1 = f(t, y)
    k2 = f(t + h * 0.5, y + k1 * h * 0.5)
    k3 = f(t + h * 0.5, y + k2 * h * 0.5)
    k4 = f(t + h, y + k3 * h)
    return y + + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

def vecLength(v1, v2, v3):
    return (v1 ** 2 + v2 ** 2 + v3 ** 2) ** 0.5

def vecDot(v1, v2, v3, b1, b2, b3):
    return v1 * b1 + v2 * b2 + v3 * b3
    
def vecCross1(v1, v2, v3, b1, b2, b3):
    return v2 * b3 - v3 * b2
def vecCross2(v1, v2, v3, b1, b2, b3):
    return v3 * b1 - v1 * b3
def vecCross3(v1, v2, v3, b1, b2, b3):
    return v1 * b2 - v2 * b1

def vecAngle(v1, v2, v3, b1, b2, b3):
    return math.acos(vecDot(v1, v2, v3, b1, b2, b3) / (vecLength(v1, v2, v3) * vecLength(b1, b2, b3)))

def vecRotation1(q0, q1, q2, q3, v1, v2, v3): #qxvxq-1

    q0inv = quatInv0(q0, q1, q2, q3)
    q1inv = quatInv1(q0, q1, q2, q3)
    q2inv = quatInv2(q0, q1, q2, q3)
    q3inv = quatInv3(q0, q1, q2, q3)

    q0int = quatProduct0(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q1int = quatProduct1(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q2int = quatProduct2(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q3int = quatProduct3(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)

    return quatProduct1(q0, q1, q2, q3, q0int, q1int, q2int, q3int)
def vecRotation2(q0, q1, q2, q3, v1, v2, v3): #qxvxq-1

    q0inv = quatInv0(q0, q1, q2, q3)
    q1inv = quatInv1(q0, q1, q2, q3)
    q2inv = quatInv2(q0, q1, q2, q3)
    q3inv = quatInv3(q0, q1, q2, q3)

    q0int = quatProduct0(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q1int = quatProduct1(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q2int = quatProduct2(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q3int = quatProduct3(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)

    return quatProduct2(q0, q1, q2, q3, q0int, q1int, q2int, q3int)
def vecRotation3(q0, q1, q2, q3, v1, v2, v3): #qxvxq-1

    q0inv = quatInv0(q0, q1, q2, q3)
    q1inv = quatInv1(q0, q1, q2, q3)
    q2inv = quatInv2(q0, q1, q2, q3)
    q3inv = quatInv3(q0, q1, q2, q3)

    q0int = quatProduct0(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q1int = quatProduct1(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q2int = quatProduct2(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)
    q3int = quatProduct3(0, v1, v2, v3, q0inv, q1inv, q2inv, q3inv)

    return quatProduct3(q0, q1, q2, q3, q0int, q1int, q2int, q3int)


def quatProduct0(p0, p1, p2, p3, q0, q1, q2, q3): #pxq
    return p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
def quatProduct1(p0, p1, p2, p3, q0, q1, q2, q3):
    return p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
def quatProduct2(p0, p1, p2, p3, q0, q1, q2, q3):
    return p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
def quatProduct3(p0, p1, p2, p3, q0, q1, q2, q3):
    return p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0

def quatLength(q0, q1, q2, q3):
    return (q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2) ** 0.5

def quatNorm0(q0, q1, q2, q3):
    return q0 / quatLength(q0, q1, q2, q3)
def quatNorm1(q0, q1, q2, q3):
    return q1 / quatLength(q0, q1, q2, q3)
def quatNorm2(q0, q1, q2, q3):
    return q2 / quatLength(q0, q1, q2, q3)
def quatNorm3(q0, q1, q2, q3):
    return q3 / quatLength(q0, q1, q2, q3)

def quatConj0(q0, q1, q2, q3):
    return q0
def quatConj1(q0, q1, q2, q3):
    return q1 * -1
def quatConj2(q0, q1, q2, q3):
    return q2 * -1
def quatConj3(q0, q1, q2, q3):
    return q3 * -1

def quatInv0(q0, q1, q2, q3):
    return quatConj0(q0, q1, q2, q3) / (quatLength(q0, q1, q2, q3) ** 2)
def quatInv1(q0, q1, q2, q3):
    return quatConj1(q0, q1, q2, q3) / (quatLength(q0, q1, q2, q3) ** 2)
def quatInv2(q0, q1, q2, q3):
    return quatConj2(q0, q1, q2, q3) / (quatLength(q0, q1, q2, q3) ** 2)
def quatInv3(q0, q1, q2, q3):
    return quatConj3(q0, q1, q2, q3) / (quatLength(q0, q1, q2, q3) ** 2)

def quatEXP0(q0, q1, q2, q3):
    return math.exp(q0) * math.cos(vecLength(q1, q2, q3))
def quatEXP1(q0, q1, q2, q3):
    return q1 * math.exp(q0) * math.sin(vecLength(q1, q2, q3)) / vecLength(q1, q2, q3)
def quatEXP2(q0, q1, q2, q3):
    return q2 * math.exp(q0) * math.sin(vecLength(q1, q2, q3)) / vecLength(q1, q2, q3)
def quatEXP3(q0, q1, q2, q3):
    return q3 * math.exp(q0) * math.sin(vecLength(q1, q2, q3)) / vecLength(q1, q2, q3)

#ryp
def eulerToQuat0(p, y, r):
    return (math.cos(p / 2) * math.cos(y / 2) * math.cos(r / 2)) + (math.sin(p / 2) * math.sin(y / 2) * math.sin(r / 2))
def eulerToQuat1(p, y, r):
    return (math.sin(p / 2) * math.cos(y / 2) * math.cos(r / 2)) - (math.cos(p / 2) * math.sin(y / 2) * math.sin(r / 2))
def eulerToQuat2(p, y, r):
    return (math.cos(p / 2) * math.sin(y / 2) * math.cos(r / 2)) + (math.sin(p / 2) * math.cos(y / 2) * math.sin(r / 2))
def eulerToQuat3(p, y, r):
    return (math.cos(p / 2) * math.cos(y / 2) * math.sin(r / 2)) - (math.sin(p / 2) * math.sin(y / 2) * math.cos(r / 2))

#ONLY IF NORMALIZED
def quatToEuler1(q0, q1, q2, q3): 
    return (q0 ** 2) - (q1 ** 2) - (q2 ** 2) + (q3 ** 2)
def quatToEuler2(q0, q1, q2, q3): 
    return math.asin(2 * ((q0 * q2) + (q3 * q1)))
def quatToEuler3(q0, q1, q2, q3): 
    return (q0 ** 2) + (q1 ** 2) - (q2 ** 2) - (q3 ** 2)

def vecProj1(v1, v2, v3, b1, b2, b3):
  return v1 - (vecDot(v1, v2, v3, b1, b2, b3) * b1)
def vecProj2(v1, v2, v3, b1, b2, b3):
  return v2 - (vecDot(v1, v2, v3, b1, b2, b3) * b2)
def vecProj3(v1, v2, v3, b1, b2, b3):
  return v3 - (vecDot(v1, v2, v3, b1, b2, b3) * b3)