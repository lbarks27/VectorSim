import numpy as np

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

import time
import math

import solvers as so

plt.style.use('dark_background')

#MISC FUNCTIONS

#MAIN VEHICLE

#whatever these values are set to are initials. x, y ,z 
linearState = np.matrix([[50, 0, 0], #pos
                        [1, 0.5, 0], #vel
                        [0, 0, 0]]) #acc

qM = [1, 0, 0, 0]
matrixM = np.matrix([[1, 0, 0], #x
                    [0, 1, 0], #y
                    [0, 0, 1]]) #z
wM = [0, 10, 0]
aM = [0, 0, 0]

#other quantities and forces
CoM = 0.5
m = 1
I = [0.1, 0.1, 0.05] #x, y, z

thrustForce = 15
thrustVector = [0, 10, -1]
thrustVectorBody = [0, 0, -1]

TESTthrustVectorBody = [0, 0, 0]
TESTthrustVector = [0, 0, 0]

projTV = [0, 0, 0]

servoXT = 0
servoYT = 0
servoXM = 0
servoYM = 0

servoXBODY = 0
servoYBODY = 0

servoSpeed = 2
servoLimit = 0.1

#control theory
Pq = 15
Pw = -9

vT = [0, 0, 1]
qT = [1, 0, 0, 0]

#RENDERING

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title(' ')

#SIM STUFF

t = 0
h = 0.01
hLaw = 0.01
g = 9.8055

plt.pause(1)

while True:

#PARAMETER CURVES

#CONTROL (streamline for microcontroller use i.e. w/o excessive function use)
    if t % hLaw < h:
        qErr = [so.quatProduct0(qT[0], qT[1], qT[2], qT[3], so.quatConj0(qM[0], qM[1], qM[2], qM[3]), so.quatConj1(qM[0], qM[1], qM[2], qM[3]), so.quatConj2(qM[0], qM[1], qM[2], qM[3]), so.quatConj3(qM[0], qM[1], qM[2], qM[3])), so.quatProduct1(qT[0], qT[1], qT[2], qT[3], so.quatConj0(qM[0], qM[1], qM[2], qM[3]), so.quatConj1(qM[0], qM[1], qM[2], qM[3]), so.quatConj2(qM[0], qM[1], qM[2], qM[3]), so.quatConj3(qM[0], qM[1], qM[2], qM[3])), so.quatProduct2(qT[0], qT[1], qT[2], qT[3], so.quatConj0(qM[0], qM[1], qM[2], qM[3]), so.quatConj1(qM[0], qM[1], qM[2], qM[3]), so.quatConj2(qM[0], qM[1], qM[2], qM[3]), so.quatConj3(qM[0], qM[1], qM[2], qM[3])), so.quatProduct3(qT[0], qT[1], qT[2], qT[3], so.quatConj0(qM[0], qM[1], qM[2], qM[3]), so.quatConj1(qM[0], qM[1], qM[2], qM[3]), so.quatConj2(qM[0], qM[1], qM[2], qM[3]), so.quatConj3(qM[0], qM[1], qM[2], qM[3]))]

        angErr = so.vecAngle(vT[0], vT[1], vT[2], matrixM[2, 0], matrixM[2, 1], matrixM[2, 2])

        vecErr = [so.vecCross1(matrixM[2, 0], matrixM[2, 1], matrixM[2, 2], vT[0], vT[1], vT[2]), so.vecCross2(matrixM[2, 0], matrixM[2, 1], matrixM[2, 2], vT[0], vT[1], vT[2]), so.vecCross3(matrixM[2, 0], matrixM[2, 1], matrixM[2, 2], vT[0], vT[1], vT[2])]
        vecErrBody = [so.vecRotation1(qM[0], qM[1], qM[2], qM[3], vecErr[0], vecErr[1], vecErr[2]), so.vecRotation2(qM[0], qM[1], qM[2], qM[3], vecErr[0], vecErr[1], vecErr[2]), so.vecRotation3(qM[0], qM[1], qM[2], qM[3], vecErr[0], vecErr[1], vecErr[2])]

        aT = [(Pq * vecErr[0]) + (Pw * wM[0]), (Pq * vecErr[1]) + (Pw * wM[1]), (Pq * vecErr[2]) + (Pw * wM[2])]

        tT = [aT[0] * I[0] + so.vecCross1(wM[0], wM[1], wM[2], wM[0] * I[0], wM[1] * I[1], wM[2] * I[2]), aT[1] * I[1] + so.vecCross2(wM[0], wM[1], wM[2], wM[0] * I[0], wM[1] * I[1], wM[2] * I[2]), aT[2] * I[2] + so.vecCross3(wM[0], wM[1], wM[2], wM[0] * I[0], wM[1] * I[1], wM[2] * I[2])]

        if (tT[0] / (thrustForce * CoM)) > 1:
            preOperation = 1
        elif (tT[0] / (thrustForce * CoM)) < -1:
            preOperation = -1
        else:
            preOperation = tT[0] / (thrustForce * CoM)

        servoXT = math.asin(preOperation)

        if (tT[1] / (thrustForce * CoM)) > 1:
            preOperation = 1
        elif (tT[1] / (thrustForce * CoM)) < -1:
            preOperation = -1
        else:
            preOperation = tT[1] / (thrustForce * CoM)

        servoYT = math.asin(preOperation)

#NAVIGATION (Motion Integration + Physics)

    #derivative of quat
    dqM = [so.quatProduct0(0, wM[0] / 2, wM[1] / 2, wM[2] / 2, qM[0], qM[1], qM[2], qM[3]), so.quatProduct1(0, wM[0] / 2, wM[1] / 2, wM[2] / 2, qM[0], qM[1], qM[2], qM[3]), so.quatProduct2(0, wM[0] / 2, wM[1] / 2, wM[2] / 2, qM[0], qM[1], qM[2], qM[3]), so.quatProduct3(0, wM[0] / 2, wM[1] / 2, wM[2] / 2, qM[0], qM[1], qM[2], qM[3])]

    #quaternion integrator
    qM = [so.quatProduct0(so.quatEXP0(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP1(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP2(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP3(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), qM[0], qM[1], qM[2], qM[3]), so.quatProduct1(so.quatEXP0(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP1(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP2(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP3(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), qM[0], qM[1], qM[2], qM[3]), so.quatProduct2(so.quatEXP0(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP1(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP2(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP3(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), qM[0], qM[1], qM[2], qM[3]), so.quatProduct3(so.quatEXP0(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP1(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP2(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), so.quatEXP3(0, 0.5 * h * wM[0], 0.5 * h * wM[1], 0.5 * h * wM[2]), qM[0], qM[1], qM[2], qM[3])]
    
    #quaternion normalization
    qM = [so.quatNorm0(qM[0], qM[1], qM[2], qM[3]), so.quatNorm1(qM[0], qM[1], qM[2], qM[3]), so.quatNorm2(qM[0], qM[1], qM[2], qM[3]), so.quatNorm3(qM[0], qM[1], qM[2], qM[3])]

    #angular velocity integration
    wM = [wM[0] + (aM[0] * h), wM[1] + (aM[1] * h), wM[2] + (aM[2] * h)]

    thrustVectorBody = [so.vecRotation1(so.eulerToQuat0((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat1((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat2((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat3((servoXM * -1), (servoYM * -1), 0), 0, 0, -1), so.vecRotation2(so.eulerToQuat0((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat1((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat2((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat3((servoXM * -1), (servoYM * -1), 0), 0, 0, -1), so.vecRotation3(so.eulerToQuat0((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat1((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat2((servoXM * -1), (servoYM * -1), 0), so.eulerToQuat3((servoXM * -1), (servoYM * -1), 0), 0, 0, -1)]
    thrustVector = [so.vecRotation1(qM[0], qM[1], qM[2], qM[3], thrustVectorBody[0], thrustVectorBody[1], thrustVectorBody[2]), so.vecRotation2(qM[0], qM[1], qM[2], qM[3], thrustVectorBody[0], thrustVectorBody[1], thrustVectorBody[2]), so.vecRotation3(qM[0], qM[1], qM[2], qM[3], thrustVectorBody[0], thrustVectorBody[1], thrustVectorBody[2])]
    tM = [CoM * thrustForce * math.sin(servoXM), CoM * thrustForce * math.sin(servoYM), 0]
    #netTorque
    netForce = [(-1 * thrustVector[0] * thrustForce), (-1 * thrustVector[1] * thrustForce), (-1 * thrustVector[2] * thrustForce)]

    linearState = np.matrix([[linearState[0, 0] + (linearState[1, 0] * h), linearState[0, 1] + (linearState[1, 1] * h), linearState[0, 2] + (linearState[1, 2] * h)], #pos
                            [linearState[1, 0] + (linearState[2, 0] * h), linearState[1, 1] + (linearState[2, 1] * h), linearState[1, 2] + (linearState[2, 2] * h)], #vel
                            [netForce[0] / m, netForce[1] / m, (netForce[2] / m) - g]]) #acc
    aM = [(tM[0] / I[0]) - (so.vecCross1(wM[0], wM[1], wM[2], I[0] * wM[0], I[1] * wM[1], I[2] * wM[2]) / I[0]), (tM[1] / I[1]) - (so.vecCross2(wM[0], wM[1], wM[2], I[0] * wM[0], I[1] * wM[1], I[2] * wM[2]) / I[1]), (tM[2] / I[2]) - (so.vecCross3(wM[0], wM[1], wM[2], I[0] * wM[0], I[1] * wM[1], I[2] * wM[2]) / I[2])]
    #aM = [(tM[0] / I[0]), (tM[1] / I[1]), (tM[2] / I[2])]

#RENDER

    matrixM = np.matrix([[so.vecRotation1(qM[0], qM[1], qM[2], qM[3], 1, 0, 0), so.vecRotation2(qM[0], qM[1], qM[2], qM[3], 1, 0, 0), so.vecRotation3(qM[0], qM[1], qM[2], qM[3], 1, 0, 0)], #x
                        [so.vecRotation1(qM[0], qM[1], qM[2], qM[3], 0, 1, 0), so.vecRotation2(qM[0], qM[1], qM[2], qM[3], 0, 1, 0), so.vecRotation3(qM[0], qM[1], qM[2], qM[3], 0, 1, 0)], #y
                        [so.vecRotation1(qM[0], qM[1], qM[2], qM[3], 0, 0, 1), so.vecRotation2(qM[0], qM[1], qM[2], qM[3], 0, 0, 1), so.vecRotation3(qM[0], qM[1], qM[2], qM[3], 0, 0, 1)]]) #z


    RM1 = ax.quiver(0, 0, 0, matrixM[0, 0] * 0.1, matrixM[0, 1] * 0.1, matrixM[0, 2] * 0.1, color='r')
    RM2 = ax.quiver(0, 0, 0, matrixM[1, 0] * 0.1, matrixM[1, 1] * 0.1, matrixM[1, 2] * 0.1, color='g')
    RM3 = ax.quiver(0, 0, 0, matrixM[2, 0], matrixM[2, 1], matrixM[2, 2], color='b')
    RM4 = ax.quiver(0, 0, 0, thrustVectorBody[0] * thrustForce, thrustVectorBody[1] * thrustForce, thrustVectorBody[2] * thrustForce, color='pink')
    RM5 = ax.quiver(0, 0, 0, vT[0], vT[1], vT[2], color='gray')
    RM6 = ax.quiver(0, 0, 0, vecErrBody[0], vecErrBody[1], vecErrBody[2], color='black')
    #RM7 = ax.quiver(0, 0, 0, TESTthrustVector[0], TESTthrustVector[1], TESTthrustVector[2], color='purple')
    RM7 = ax.quiver(0, 0, 0, TESTthrustVector[0], TESTthrustVector[1], TESTthrustVector[2], color='purple')

    #RM6 = ax.quiver(0, 0, 0, linearState[0, 0], linearState[0, 1], linearState[0, 2], color='lightblue')

    plt.pause(0.00001)
    plt.draw()
    print( )
    print( )
    print( )
    print( )
    print( )
    print( )
    print( )
    print( )
    print( )
    print( )
    print(qM)

    RM1.remove()
    RM2.remove()
    RM3.remove()
    RM4.remove()
    RM5.remove()
    RM6.remove()
    RM7.remove()
    
    t += h

    #ERROR

    servoXM = servoXT
    servoYM = servoYT

    if servoXM > servoLimit:
        servoXM = servoLimit
    if servoXM < (servoLimit * -1):
        servoXM = (servoLimit * -1)

    if servoYM > servoLimit:
        servoYM = servoLimit
    if servoYM < (servoLimit * -1):
        servoYM = (servoLimit * -1)

    if 5 < t < 10:
        vT = [0, 1, 1]
    if 10 < t < 15:
        vT = [1, 0, -1]
    if 15 < t < 20:
        vT = [0, -1, 1]
    if 20 < t < 25:
        vT = [-1, 0, -1]
    if 25 < t < 30:
        vT = [0, 0, 1]
    if t > 30:
        thrustForce = 6
    if 30 < t < 35:
        vT = [0, 0, 1]
    if 35 < t < 40:
        vT = [0, 1, -1]
    if 40 < t < 45:
        vT = [1, 0, 1]
    if 45 < t < 50:
        vT = [0, -1, -1]
    if 50 < t < 55:
        vT = [-1, 0, 1]
    if 55 < t < 60:
        vT = [0, 0, 1]
plt.show