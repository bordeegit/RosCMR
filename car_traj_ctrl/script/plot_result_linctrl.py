import sys
import rosbag
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy.integrate as numint

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the simulator
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_velocity = []
vehicleState_steer = []

# State published by the controller
controllerState_time = []
controllerState_vPx = []
controllerState_vPy = []
controllerState_velocity = []
controllerState_steer = []

# Trajectory generated
traj_x = []
traj_y = []

for topic, msg, t in bag.read_messages():
    if topic == "/car_state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_velocity.append(msg.data[4])
        vehicleState_steer.append(msg.data[5])

    if topic == "/controller_state":
        controllerState_time.append(msg.data[0])
        controllerState_vPx.append(msg.data[1])
        controllerState_vPy.append(msg.data[2])
        controllerState_velocity.append(msg.data[3])
        controllerState_steer.append(msg.data[4])

    if topic == "/ref_traj":
        traj_x.append(msg.data[1])
        traj_y.append(msg.data[2])

bag.close()
"""
Trajectory Data
a = 2
T = 3.8
xref = a*np.sin(np.multiply(2*math.pi/T,vehicleState_time))
yref = a*np.multiply(np.sin(np.multiply(2*math.pi/T,vehicleState_time)),
                                 np.cos(np.multiply(2*math.pi/T,vehicleState_time)))
"""

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y, label = "actual")
#plt.plot(xref,yref,  'r--', label = "ref")
plt.plot(traj_x,traj_y, 'g--', label = "gen")
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.plot(traj_x[len(traj_x)-1],traj_y[len(traj_y)-1], 'yx')
plt.legend(loc="best")
plt.xlabel("x [m]")
plt.ylabel("y [m]")

"""
plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_velocity)
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_steer)
plt.xlabel("Time [s]")
plt.ylabel("Steer position [rad]")
"""

plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, label="actual")
plt.plot(controllerState_time[:len(controllerState_time)],traj_x,'r--', label="ideal")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend(loc="best")
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, label="actual")
plt.plot(controllerState_time[:len(controllerState_time)],traj_y,'r--', label="ideal")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend(loc="best")
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

"""
plt.figure(4)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_vPx)
plt.xlabel("Time [s]")
plt.ylabel("Point P x velocity [m/s]")
plt.subplot(212)
plt.plot(controllerState_time,controllerState_vPy)
plt.xlabel("Time [s]")
plt.ylabel("Point P y velocity [m/s]")
"""

#print(len(vehicleState_x)/10)
#print(len(controllerState_vPx))

#Changed index of vehicleState_x/y to match with controllerState size
#   this will lose some values of vehicleState or add some 0s, but it fixes the issue

#TODO: problem here, if i slice the index of vehicle state by 10 the timing doesn't match, so we get constant error
"""
index = len(vehicleState_x[::10])-1
print(vehicleState_time[index*10])
print(controllerState_time[index])

err_x = [abs(v - t) for v, t in zip(vehicleState_x[:len(traj_x)], traj_x)]
err_y = [abs(v - t) for v, t in zip(vehicleState_y[:len(traj_y)], traj_y)]

plt.figure(5)
plt.subplot(211)
plt.plot(controllerState_time,err_x)
plt.xlabel("Time [s]")
plt.ylabel("Error on x [m]")
plt.subplot(212)
plt.plot(controllerState_time,err_y)
plt.xlabel("Time [s]")
plt.ylabel("Error on y [m]")
"""

plt.show(block=False)

raw_input("Press Enter to End")
