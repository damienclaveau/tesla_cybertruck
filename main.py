def loop_update_sensors():
    global compass_heading
    # Update Compass direction, current speed, deviation, commands coming from Bluetooth, ...
    # Compute current position
    compass_heading = input.compass_heading()
    pass

def loop_update_vision():
    # Capture a new video frame and analyze it : is there 1 ball, no ball, a Tag, an obstacle or nothing ?
    huskylens.request()
    #######################################################
    # mode = ALGORITHM_OBJECT_TRACKING
    if huskylens.is_appear(1, HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK):
        target_x = huskylens.reade_box(1, Content1.X_CENTER);
        target_y = huskylens.reade_box(1, Content1.Y_CENTER);
    #######################################################
    # mode = OBJECTCLASSIFICATION
    # global object_list
    # object_list = [
    #       huskylens.readBox_ss(1, Content3.ID),
    #       huskylens.readBox_ss(2, Content3.ID),
    #       huskylens.readBox_ss(3, Content3.ID),
    #       huskylens.readBox_ss(4, Content3.ID)]
    # basic.show_string("" + str(object_list[0]) + ("" + str(object_list[1])) + ("" + str(object_list[2])) + ("" + str(object_list[3])))
    #######################################################
    # mode= ALGORITHM_COLOR_RECOGNITION
    # if huskylens.readBox_s(Content3.ID) == 1: # "Color ID=1"
    pass

def loop_compute_waypoint():
    # Based on the Robot State, Goal and Environment
    # Determine the next waypoint to reach, and linear and angular velocities to reach it
    pass

def loop_motor_controller_run():
    # Drive servo and motor with PWM according to updated linear and angular velocities
    # Set the steering servo position to aim to the waypoint
    steering = 0 # to be computed with PID
    if exec_mode == "makecode":
        servos.P0.set_angle(steering)
    else:
        amaker_motor.servo_position(amaker_motor.Servos.S1, steering)
    # Set the servo throttle power depending on the remainning distance to the waypoint
    pwm = 50 # to be computed
    if exec_mode == "makecode":
        servos.P1.run(pwm)
    else:
        amaker_motor.servo_speed(amaker_motor.Servos.S3, pwm)

def loop_state_machine():
    # Based on input signals, current position, object recgnition and game instructions
    # determine the next action and next state
    huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    UTBBot.new_bot_status(UTBBotCode.BotStatus.IDLE)
    pass
    
def loop_feedback():
    # Provide feedback to exterior using display and radio
    basic.show_arrow(ArrowNames.NORTH)
    UTBBot.emit_status()
    UTBBot.emit_heart_beat()
    pass

def loop_telemetry():
    # Log metrics and update Display
    #   huskylens.write_osd("angle diff "+str(steering_angle), 150, 30)
    #   huskylens.write_osd("servo_angle "+str(servo_angle), 150, 60)
    #   huskylens.write_osd("input "+str(steering_value), 150, 90)
    huskylens.write_osd("head "+str(compass_heading), 150, 120)
    datalogger.log(
    #       datalogger.create_cv("target_pos_dist", target_pos_dist)
    #       ,datalogger.create_cv("target_pos_theta", target_pos_theta)
    #       ,datalogger.create_cv("velocity_speed", velocity_speed)
    #       ,datalogger.create_cv("velocity_angle ", velocity_angle )
            datalogger.create_cv("compass_heading", compass_heading)
        )


cycles_count = 0
compass_heading = 0
avg_loop_rate = 0
runtime_ms = 0
exec_mode = "makecode"

def init():
    # Boot sequence
    # Initialization of the sensors, variables, display, callbacks
    basic.clear_screen()
    huskylens.init_i2c()
    huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    input.calibrate_compass()
    input.set_accelerometer_range(AcceleratorRange.ONE_G)
    datalogger.set_column_titles("target_pos_dist", "target_pos_theta", "velocity_speed", "velocity_angle", "compass_heading")
    pass

init()

def on_forever():
    global cycles_count
    # Infinite loop (frequency = ?? Hz)
    cycles_count += 1
    loop_update_sensors()
    loop_update_vision()
    loop_state_machine()
    loop_compute_waypoint()
    loop_motor_controller_run()
    loop_feedback()
    loop_telemetry()
# best effort loop
basic.forever(on_forever)

# scheduled function calls
def on_every_100ms():
    pass
def on_every_200ms():
    pass
def on_every_500ms():
    pass

loops.every_interval(100, on_every_100ms)
loops.every_interval(200, on_every_200ms)
loops.every_interval(500, on_every_500ms)
