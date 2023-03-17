# LEGO type:standard slot:3

import time
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *


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


def pid_yaw_angle(heading: int, amount: int | float,
                  condition: str = 'degrees', speed: int = 60,
                  kp: float = 1.4, ki: float = 0.01, kd: float = 2,
                  accel: bool = True, a: float = 0.7, decel: bool = False,
                  ad: float = 0.5, decelt: float | int = 80) -> None:
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


def main():
    hub.motion_sensor.reset_yaw_angle()
    # move towards
    # motors.move(1600, 'degrees', speed=80)
    pid_yaw_angle(0, 1600, speed=80)
    # leave energy units
    up_left.run_for_degrees(1260, 70)
    up_left.run_for_degrees(200, -70)
    # move towards toy factory
    motors.move(300, 'degrees', speed=-80)
    gyro_turn(90, 40, True, False)
    pid_yaw_angle(90, 1200, speed=80)
    gyro_turn(-130, -40, False, True)
    motors.move(240, 'degrees', speed=90)
    # drop remaining energy unit
    up_left.run_for_degrees(500, 70)
    gyro_turn(60, -40, True, False)
    motors.move(60, 'degrees', speed=-80)
    # move towards
    # pid_yaw_angle(130, 800, speed=80)
    motors.move(800, 'degrees', speed=80)
    gyro_turn(40, -40, False, False)
    # pid_yaw_angle(160, 700, speed=80)
    motors.move(700, 'degrees', speed=80)
    up_right.run_for_degrees(150, 100)
    # drop energy units
    motors.move(100, 'degrees', speed=95)
    motors.move(300, 'degrees', speed=90)
    gyro_turn(100, -40, False, False)
    # pid_yaw_angle(90, 900, speed=100)
    motors.move(900, 'degrees', speed=100)
    gyro_turn(90, -40, False, False)


def gyro_square_off(heading: int):
    while (cur := hub.motion_sensor.get_yaw_angle()) != heading:
        sign = 1 if cur - heading < 0 else -1
        motors.start_tank(sign * 10, -sign * 10)
    motors.stop()


def main_v2():
    # got to the energy storage
    hub.motion_sensor.reset_yaw_angle()
    pid_yaw_angle(0, 1000, speed=70)
    gyro_turn(45, 30, True, False)
    pid_yaw_angle(45, 200, speed=70)
    gyro_turn(0, 30, False, True)
    pid_yaw_angle(0, 175, speed=40)
    # drop the energy
    up_left.run_for_degrees(1200, 70)
    up_left.run_for_degrees(200, -70)
    # go to factory
    motors.move(300, 'degrees', speed=-70)
    gyro_turn(90, 40, True, False)
    pid_yaw_angle(90, 1000, speed=70)
    gyro_turn(135, 30, True, False)
    pid_yaw_angle(135, 350, speed=50)
    hub.motion_sensor.reset_yaw_angle()
    # motors.move(400, 'degrees', speed=50)
    # drop last energy unit
    up_left.run_for_degrees(350, 70)
    up_left.run_for_degrees(500, -70)
    # move towards power plant
    # motors.move(500, 'degrees', speed=-70)
    pid_yaw_angle(0, 420, speed=-60)
    # gyro_turn(46, 30, True, False)
    gyro_turn(50, 30, True, False)
    # motors.move(880, 'degrees', speed=70)
    pid_yaw_angle(48, 815, speed=70)
    up_right.run_for_degrees(300, 100)
    wait_for_seconds(1)
    motors.move(150, 'degrees', speed=50)
    # pid_yaw_angle(179, 870, speed=80)
    # move arm up
    # drop energy units
    # motors.move(100, 'degrees', speed=90)'''
    # move towards home
    motors.move(190, 'degrees', speed=-60)
    gyro_turn(-25, 50, False, False)
    motors.move(1500, 'degrees', speed=90)
    # gyro_turn(0, 30, False, False)
    # motors.move(800, 'degrees', speed=90)
    # move back from power plant
    # motors.move(200, 'degrees', speed=-40)
    # turn to launch area
    # gyro_turn(-90, -40, False, False)
    # move forward to launch area
    # motors.move(1600, 'degrees', speed=60)
    raise SystemExit


# main()
main_v2()
