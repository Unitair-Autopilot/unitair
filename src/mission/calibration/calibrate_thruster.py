#!/usr/bin/env python3
"""
Fits the following monotone-quadratic thruster model to a CSV of thrust and back-torque data.
    thrust = (c1 + c2*|effort|)*effort
    torque = cr*thrust

In relation to the Allocator, this corresponds to an Actuator with:
    coeff1 = c1 / ||linvec||
    coeff2 = c2 / ||linvec||
    angvec = cr * +-1(handedness) * linvec

The path to the CSV must be provided as an argument to this script.

The data collected should be a slow ramp-up from minimum to maximum effort and provide:
    Effort as 'ESC_signal_µs'
    Thrust as 'Thrust_kgf'
    Torque as 'Torque_N·m'

If your column names or units differ, you'll have to convert the data or edit this script.

The outputted model will be in base-SI units (N, m) and normalized (unitless) effort.

"""
from sys import argv
import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot

name = "calibrate_thruster"
print("")

if len(argv) != 2:
    raise IOError(name+": path to data file was not provided")
else:
    file = argv[1]

################################################## (EDIT COLUMN NAMES HERE)

print(name+": loading data from '{}'...".format(file))

data = np.genfromtxt(file, dtype=float, delimiter=',', names=True)
n = len(data)

efforts = data['ESC_signal_µs']
thrusts = data['Thrust_kgf']
torques = data['Torque_N·m']

# First entry with thrust within 5% of max
peak = ((thrusts.max()-thrusts) <= 0.05*abs(thrusts.max())).argmax()

################################################## (EDIT MODEL UNITS HERE)

print(name+": adjusting units...")

efforts -= efforts.min()
efforts /= efforts[peak]

thrusts *= 9.80665

##################################################

print(name+": fitting...")

m = 3

def model(efforts, parameters):
    predicted_thrusts = (parameters[0] + parameters[1]*np.abs(efforts))*efforts
    predicted_torques = parameters[2]*predicted_thrusts
    return (predicted_thrusts, predicted_torques)

def discrepancy(parameters):
    predicted_thrusts, predicted_torques = model(efforts, parameters)
    errors = np.concatenate((predicted_thrusts - thrusts, predicted_torques - torques))
    return np.sum(np.square(errors)) / n

parameters = minimize(discrepancy, np.zeros(m, float)).x

print("\n==========================")
print("CALIBRATE THRUSTER RESULTS")
print("==========================")
print("c1:", parameters[0], "[N/eff]")
print("c2:", parameters[1], "[N/eff^2]")
print("cr:", parameters[2], "[N*m/N]")
print("==========================\n")

##################################################

print(name+": interpolating...")

resolution = 0.001
domain = np.arange(0.0, 1.0+resolution, resolution)
predicted_thrusts, predicted_torques = model(domain, parameters)

print(name+": plotting for visual inspection of fit...")

pyplot.rcParams["font.size"] = 18
pyplot.rcParams["axes.grid"] = True

figure, axes = pyplot.subplots(1,2)
figure.suptitle("Calibrate Thruster Results")

axes[0].set_xlabel("Effort")
axes[0].set_ylabel("Thrust [N]")
axes[0].scatter(efforts, thrusts, color='r')
axes[0].plot(domain, predicted_thrusts, color='b')

axes[1].set_xlabel("Thrust [N]")
axes[1].set_ylabel("Torque [N*m]")
axes[1].scatter(thrusts, torques, color='r')
axes[1].plot(predicted_thrusts, predicted_torques, color='b')

print(name+": close plot to finish...\n")
pyplot.show()

##################################################
