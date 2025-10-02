function loop_update_sensors() {
    
    //  Update Compass direction, current speed, deviation, commands coming from Bluetooth, ...
    //  Compute current position
    compass_heading = input.compassHeading()
    
}

function loop_update_vision() {
    let target_x: number;
    let target_y: number;
    //  Capture a new video frame and analyze it : is there 1 ball, no ball, a Tag, an obstacle or nothing ?
    huskylens.request()
    // ######################################################
    //  mode = ALGORITHM_OBJECT_TRACKING
    if (huskylens.isAppear(1, HUSKYLENSResultType_t.HUSKYLENSResultBlock)) {
        target_x = huskylens.readeBox(1, Content1.xCenter)
        target_y = huskylens.readeBox(1, Content1.yCenter)
    }
    
    // ######################################################
    //  mode = OBJECTCLASSIFICATION
    //  global object_list
    //  object_list = [
    //        huskylens.readBox_ss(1, Content3.ID),
    //        huskylens.readBox_ss(2, Content3.ID),
    //        huskylens.readBox_ss(3, Content3.ID),
    //        huskylens.readBox_ss(4, Content3.ID)]
    //  basic.show_string("" + str(object_list[0]) + ("" + str(object_list[1])) + ("" + str(object_list[2])) + ("" + str(object_list[3])))
    // ######################################################
    //  mode= ALGORITHM_COLOR_RECOGNITION
    //  if huskylens.readBox_s(Content3.ID) == 1: # "Color ID=1"
    
}

function loop_compute_waypoint() {
    //  Based on the Robot State, Goal and Environment
    //  Determine the next waypoint to reach, and linear and angular velocities to reach it
    
}

function loop_motor_controller_run() {
    //  Drive servo and motor with PWM according to updated linear and angular velocities
    //  Set the steering servo position to aim to the waypoint
    let steering = 0
    //  to be computed with PID
    if (exec_mode == "makecode") {
        servos.P0.setAngle(steering)
    } else {
        amaker_motor.servoPosition(amaker_motor.Servos.S1, steering)
    }
    
    //  Set the servo throttle power depending on the remainning distance to the waypoint
    let pwm = 50
    //  to be computed
    if (exec_mode == "makecode") {
        servos.P1.run(pwm)
    } else {
        amaker_motor.servoSpeed(amaker_motor.Servos.S3, pwm)
    }
    
}

function loop_state_machine() {
    //  Based on input signals, current position, object recgnition and game instructions
    //  determine the next action and next state
    huskylens.initMode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    UTBBot.newBotStatus(UTBBotCode.BotStatus.Idle)
    
}

function loop_feedback() {
    //  Provide feedback to exterior using display and radio
    basic.showArrow(ArrowNames.North)
    UTBBot.emitStatus()
    UTBBot.emitHeartBeat()
    
}

function loop_telemetry() {
    //  Log metrics and update Display
    //    huskylens.write_osd("angle diff "+str(steering_angle), 150, 30)
    //    huskylens.write_osd("servo_angle "+str(servo_angle), 150, 60)
    //    huskylens.write_osd("input "+str(steering_value), 150, 90)
    huskylens.writeOSD("head " + ("" + compass_heading), 150, 120)
    datalogger.log(datalogger.createCV("compass_heading", compass_heading))
}

//        datalogger.create_cv("target_pos_dist", target_pos_dist)
//        ,datalogger.create_cv("target_pos_theta", target_pos_theta)
//        ,datalogger.create_cv("velocity_speed", velocity_speed)
//        ,datalogger.create_cv("velocity_angle ", velocity_angle )
let cycles_count = 0
let compass_heading = 0
let avg_loop_rate = 0
let runtime_ms = 0
let exec_mode = "makecode"
function init() {
    //  Boot sequence
    //  Initialization of the sensors, variables, display, callbacks
    basic.clearScreen()
    huskylens.initI2c()
    huskylens.initMode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    input.calibrateCompass()
    input.setAccelerometerRange(AcceleratorRange.OneG)
    datalogger.setColumnTitles("target_pos_dist", "target_pos_theta", "velocity_speed", "velocity_angle", "compass_heading")
    
}

init()
//  best effort loop
basic.forever(function on_forever() {
    
    //  Infinite loop (frequency = ?? Hz)
    cycles_count += 1
    loop_update_sensors()
    loop_update_vision()
    loop_state_machine()
    loop_compute_waypoint()
    loop_motor_controller_run()
    loop_feedback()
    loop_telemetry()
})
//  scheduled function calls
loops.everyInterval(100, function on_every_100ms() {
    
})
loops.everyInterval(200, function on_every_200ms() {
    
})
loops.everyInterval(500, function on_every_500ms() {
    
})
