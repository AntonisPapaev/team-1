#PID PYTHON

import numpy as np
import time
import matplotlib.pyplot as plt
from opencv_functions import Image


img = Image()
img.find_error_from_middle


def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt, image_obj):
    error = image_obj.find_error_from_middle() # PV is the real-time measured value of the system parameter i want to control, (In my case this will be distance from center i think) here it is subtracting the Process Variable from the desired set point to find the real-time error
    integral += error * dt # Integral here is adding the errors every time step (i think) the time step is represented by dt. As the integral term accumulates over time, the 'urge' to return the error to zero increases.
    derivative = (error - previous_error) / dt # Looks like acceleration formula!! rate of change of error
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

def main():
    
    setpoint = 50 # The set point desired
    pv = 6 # Initial process variable (consider implementing a function that makes it oscilate over time so i can see PID control in action)
    kp = 1.55 # Proportional Gain (determines how strongly the output reacts to the current error by agressively correcting)
    ki = 0.7 # Integral Gain (determines how agressively the controller ramps up the correction when the error won't go away)
    kd = 0.50 # Derivative Gain (determines how strong the preventative correction value will be)
    previous_error = 0
    integral = 0
    dt = 0.1 # Time step

    time_steps = [] # Stores time points at each iteration
    pv_values = [] # Stores process variable at each time step
    control_values = [] # Stores control output at each time step
    setpoint_values = [] # Stores the setpoint value at each time step

    drift_per_step = 10 # Disturbance size that moves PV away from the setpoint each iteration

    for i in range(200):    # Simulate for 100 time steps/increments
        control, error, integral = pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt) #in each iteration the pid_controller function is called to compute the control output
        pv += control * dt  # Updates the value of the process variable based on the value of the control output (corrective force)
        if pv < setpoint:
            pv -= drift_per_step
        else:
            pv += drift_per_step
        previous_error = error

        time_steps.append(i * dt)
        pv_values.append(pv)
        control_values.append(control)
        setpoint_values.append(setpoint)

        time.sleep(dt)


    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_steps, pv_values, label='Process Variable (PV)')
    plt.plot(time_steps, setpoint_values, label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Process Variable vs. Setpoint')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time_steps, control_values, label='Control Output')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Output')
    plt.title('Control Output over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
        main()
