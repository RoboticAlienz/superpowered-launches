# LEGO type:standard slot:4

import time
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer  # type: ignore
from math import *  # type: ignore


def time_f(func):

    def wrapper(*args, **kwargs):
        st = time.time()
        try:
            return func(*args, **kwargs)
        finally:
            print('{​​}​​ took {​​}​​ secs'.format(
                func.__name__, time.time() - st))

    return wrapper


hub = PrimeHub()
motors = MotorPair('F', 'B')
left = Motor('F')
right = Motor('B')

up_right = Motor('D')
up_left = Motor('C')

l_col = ColorSensor('A')
r_col = ColorSensor('E')


def gyro_turn(target: int, speed: int, right: bool, right_wheel: bool) -> None:
    if right:
        if right_wheel:
            motors.start_tank(left_speed=0, right_speed=-speed)
        else:
            motors.start_tank(left_speed=speed, right_speed=0)
        while hub.motion_sensor.get_yaw_angle() < target:
            pass
    else:
        if right_wheel:
            motors.start_tank(left_speed=0, right_speed=speed)
        else:
            motors.start_tank(left_speed=-speed, right_speed=0)
        while hub.motion_sensor.get_yaw_angle() > target:
            pass
    motors.stop()


def accelerate(amount: int, start_speed: int, end_speed: int) -> None:
    step = 1 if start_speed > 0 else -1
    amount = abs(amount)
    left.set_degrees_counted(0)
    right.set_degrees_counted(0)
    for i in range(start_speed, end_speed + 1, step):
        motors.start(speed=i)
        wait_for_seconds(0.04)
        if abs(-left.get_degrees_counted() + right.get_degrees_counted()) / 2 < amount:
            break
    while abs(-left.get_degrees_counted() + right.get_degrees_counted()) / 2 < amount:
        pass
    motors.stop()


def stall(speed: int) -> None:
    motors.start(speed=speed)
    while -left.get_speed() <= 30 or right.get_speed() <= 30:
        pass
    while -left.get_speed() > 25 or right.get_speed() > 25:
        pass
    motors.stop()


def stall_up(speed: int):
    motors.start(speed=speed)
    while up_left.get_speed() <= 45:
        pass
    while up_left.get_speed() > 40:
        pass
    motors.stop()


def color_wait(speed: int) -> None:
    motors.start(speed=speed)
    while l_col.get_reflected_light() > 50 and r_col.get_reflected_light() > 50:
        pass
    motors.stop()


def pid_yaw_angle(heading: int, amount: int | float, #type: ignore
                  condition: str = 'degrees', speed: int = 60,
                  kp: float = 1.4, ki: float = 0.01, kd: float = 2,
                  accel: bool = True, a: float = 0.7, decel: bool = False,
                  ad: float = 0.5, decelt: float | int = 80) -> None: #type: ignore
    # Cache references locally (optimization)
    yaw = hub.motion_sensor.get_yaw_angle
    pair_s = motors.start_tank_at_power
    fabs_ = fabs
    pair_s(speed, speed)
    prev = 0
    i = 0
    # If condition is degrees
    if condition == 'degrees':
        # Cache more references locally (optimization)
        l = left.get_degrees_counted
        r = right.get_degrees_counted
        # Get starting degrees for both driving motors
        stl = l()
        str_ = r()
        # Set acceleration and deceleration rates according to the sign of
        # the given speed
        num = a if speed > 0 else -a
        numd = -ad if speed > 0 else ad
        # Initialize speed to the acceleration rate (will be overriden if
        # accel is False)
        spd = num
        # If accel is False
        if not accel:
            # Start driving motors at given speed
            pair_s(speed, speed)
            # Set spd to given speed
            spd = speed
        # While the average |position difference of both driving motors
        # between their current and starting values| is less than the given
        # amount
        while (cur := (fabs_(l() - stl) + fabs_(r() - str_))/2) < amount:
            # If accel is True and the absolute value of the current speed
            # is less than that of the given speed
            if accel and fabs_(spd) < fabs_(speed):
                # Add acceleration rate to spd
                spd += num
                n = round(spd)
                # Set the driving motors' speed to the rounded value of
                # spd, since MotorPair.start_tank_at_power only accepts
                # ints
                pair_s(n, n)
            # If decel is True and degrees remaining are less than decelt
            # and |spd| is less than 10
            if decel and amount - cur < decelt and fabs_(spd) > 10:
                # Set accel to False (end acceleration if it hasn't already
                # ended)
                accel = False
                # Add deceleration rate to spd
                spd += numd
                n = round(spd)
                # Set the driving motors' speed to the rounded value of
                # spd, since MotorPair.start_tank_at_power only accepts
                # ints
                pair_s(n, n)
            # Simple implementation of PID algorithm
            # Error = given heading - current heading
            er = heading - yaw()
            # Integral = integral + error * ki
            i += er*ki
            # Derivative = error - previous error
            d = er - prev
            # Final steering is all terms multiplied by their coefficients
            # summed up
            steer = er*kp + i + d*kd
            # Set driving motor speeds by adding or subtracting the
            # steering value from current speed
            pair_s(round(spd + steer), round(spd - steer))
            # Previous error = current error
            prev = er
        # Stop driving motors
        motors.stop()
    # If condition is lp (light port)
    elif condition == 'lp':
        # Cache reference locally (optimization)
        l = l_col.get_reflected_light
        # Scale and shift calibrated range to fit raw range
        # amount = (amount*HIGH_M_LOW//100) + LOW
        # If amount is greater than 50
        if amount < 50:
            # While current reflected light is greater than given amount
            while l() > amount:
                # Do PID stuff
                er = heading - yaw()
                i += er*ki
                d = er - prev
                steer = er*kp + i + d*kd
                pair_s(round(speed + steer), round(speed - steer))
                prev = er
            # Stop driving motors
            motors.stop()
            # Return to avoid another if statement
            return
        # If we have reached this point, that means that amount >= 50.
        # While current reflected light is less than given amount
        while l() < amount:
            # Do PID stuff
            er = heading - yaw()
            i += er*ki
            d = er - prev
            steer = er*kp + i + d*kd
            pair_s(round(speed + steer), round(speed - steer))
            prev = er
        # Stop driving pair
        motors.stop()
    # If condition is ls (light starboard), similar to lp
    elif condition == 'ls':
        l = r_col.get_reflected_light
        # amount = (amount*HIGH_M_LOW//100) + LOW
        if amount < 50:
            while l() > amount:
                er = heading - yaw()
                i += er*ki
                d = er - prev
                steer = er*kp + i + d*kd
                pair_s(round(speed + steer), round(speed - steer))
                prev = er
            motors.stop()
            return
        while l() < amount:
            er = heading - yaw()
            i += er*ki
            d = er - prev
            steer = er*kp + i + d*kd
            pair_s(round(speed + steer), round(speed - steer))
            prev = er
        motors.stop()

def main_try():
    hub.motion_sensor.reset_yaw_angle()
    gyro_turn(-90, 40, False, True)
    motors.move(100, 'degrees', speed=90)

def main_test():
    hub.motion_sensor.reset_yaw_angle()
    gyro_turn(-90, 50, False, True)



# main_try()

main_test()