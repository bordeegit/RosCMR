import sys
import rosbag
import matplotlib.pyplot as plt
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

bag.close()

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y)
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_velocity)
plt.xlabel("Time [s]")
plt.ylabel("Longitudinal velocity [m/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_steer)
plt.xlabel("Time [s]")
plt.ylabel("Steer position [rad]")

plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, label="actual")
plt.plot(controllerState_time[:len(controllerState_time)-1],numint.cumtrapz(controllerState_vPx, dx=0.01),'r--', label="ideal")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend(loc="best")
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, label="actual")
plt.plot(controllerState_time[:len(controllerState_time)-1],numint.cumtrapz(controllerState_vPy, dx=0.01),'r--', label="ideal")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend(loc="best")
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

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
plt.figure(5)
plt.subplot(211)
plt.plot(controllerState_time[:len(controllerState_time)-1],vehicleState_x[:len(vehicleState_x):10]-numint.cumtrapz(controllerState_vPx, dx=0.01))
plt.xlabel("Time [s]")
plt.ylabel("Point P x velocity [m/s]")
plt.subplot(212)
plt.plot(controllerState_time[:len(controllerState_time)-1],vehicleState_y[:len(vehicleState_x):10]-numint.cumtrapz(controllerState_vPy, dx=0.01))
plt.xlabel("Time [s]")
plt.ylabel("Point P y velocity [m/s]")
"""

plt.show(block=False)

raw_input("Press Enter to End")