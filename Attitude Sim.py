

import numpy as np
import quaternion as quat
import solvers_updated as so

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import time

import render as re
import physics as ph


####################################################################################################################################################################################################################################################################################################################


#simulation helpers
null_rotation_matrix = np.array([[1, 0, 0], #x
                                 [0, 1, 0], #y
                                 [0, 0, 1]]) #z

#simulation parameters
t = 0

sim_step = 0.01

render_rate = 0
sim_loops_per_render = 0

#render parameters
sim_center = np.array([0, 0, 0])


####################################################################################################################################################################################################################################################################################################################


#OOP setup

#note where parameters are iterated

#linear_inertia: 

class Craft:
    def __init__(self, linear_inertia = 1, attitude_inertia = np.array([1, 0.5, 0.1]), torque_meas = np.array([0, 0, 0]), force_meas = np.array([0, 0, 0]), attitude_quaternion_meas = quat.quaternion(1, 0, 0, 0), attitude_velocity_meas = np.array([0.0001, 0.0001, 0.0001], dtype = np.float64), linear_position_meas = np.array([0, 0, 0]), linear_velocity_meas = np.array([0, 0, 0])):
        
        #rotational parameters
        self.torque_meas = torque_meas
        self.attitude_inertia = attitude_inertia

        #integrated rotational motion
        self.attitude_quaternion_meas = attitude_quaternion_meas
        self.attitude_quaternion_derivative_meas = quat.quaternion(0, attitude_velocity_meas[0] / 2, attitude_velocity_meas[1] / 2, attitude_velocity_meas[2] / 2) * attitude_quaternion_meas
        self.attitude_velocity_meas = attitude_velocity_meas
        #ODE rotational motion

        self.attitude_state = np.array([torque_meas, attitude_inertia, attitude_velocity_meas])


        #linear parameters
        self.linear_inertia = linear_inertia
        self.force_meas = force_meas

        #integrated linear motion
        self.linear_position_meas = linear_position_meas
        self.linear_velocity_meas = linear_velocity_meas

        #ODE rotational motion
        def linear_acceleration_meas(self, force_meas, linear_inertia):
            return (force_meas / linear_inertia)


    class Motor:
        #will need to include self_vector and angle_relative_to_body later
        def __init__(self, body_position, body_vector, thrust_force, body_angle):
            pass

        class Servo: 
            def __init__(self) -> None:
                pass


####################################################################################################################################################################################################################################################################################################################


#simulation initialization

main_craft = Craft()

#render start

plt.style.use('dark_background')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2) # type: ignore

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z') # type: ignore
ax.set_title(' ')


####################################################################################################################################################################################################################################################################################################################


#simulation loop


while True:

    #physics integration update
    t = t + sim_step
#MOVE TO UPDATE DEF IN CLASS
    main_craft.attitude_velocity_meas = np.reshape(main_craft.attitude_velocity_meas, (3, ))
    main_craft.attitude_state = np.array([main_craft.torque_meas, main_craft.attitude_inertia, main_craft.attitude_velocity_meas])
    
    #attitude first integration
    main_craft.attitude_velocity_meas = np.reshape(main_craft.attitude_velocity_meas, (1, 3))
    main_craft.attitude_velocity_meas += ph.euler_step(ph.second_order_attitude_ode, t, main_craft.attitude_state, sim_step)

    #attitude second integration
    main_craft.attitude_quaternion_meas = so.quat_norm(main_craft.attitude_quaternion_meas)
    main_craft.attitude_quaternion_meas += so.quat_exp(quat.quaternion(0, 0.5 * main_craft.attitude_velocity_meas[0][0], 0.5 * main_craft.attitude_velocity_meas[0][1], 0.5 * main_craft.attitude_velocity_meas[0][2])) * main_craft.attitude_quaternion_meas

    #control update

    main_craft.torque_meas = np.array([np.sin(t), 0, 0])

    #guidance update


    #render update

    ax.clear()

    matrix_render_aid = np.array(re.rotation_matrix(main_craft.attitude_quaternion_meas)[0])
    RM1 = ax.quiver(0, 0, 0, matrix_render_aid[0] * 0.1, matrix_render_aid[1] * 0.1, matrix_render_aid[2] * 0.1, color='r')
    matrix_render_aid = np.array(re.rotation_matrix(main_craft.attitude_quaternion_meas)[1])
    RM2 = ax.quiver(0, 0, 0, matrix_render_aid[0] * 0.1, matrix_render_aid[1] * 0.1, matrix_render_aid[2] * 0.1, color='g')
    matrix_render_aid = np.array(re.rotation_matrix(main_craft.attitude_quaternion_meas)[2])
    RM3 = ax.quiver(0, 0, 0, matrix_render_aid[0], matrix_render_aid[1], matrix_render_aid[2], color='b')

    plt.pause(0.001)
    plt.draw()

    #telemetry update

    print(main_craft.attitude_state[1])
    print(t)