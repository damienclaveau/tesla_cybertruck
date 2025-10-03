hl_mode = 0

GAME_DURATION = 300 # seconds
TIME_TO_GO_HOME = 20 # seconds
OBJECT_LOST_DELAY = 1 # second

class ExecMode:
    MakeCode = 1 # Offline software development with no attached board
    LiveMode = 2 # Free running Robot
    WiredMode = 3 # Board wired with USB

EXEC_MODE = ExecMode.MakeCode

HUSKY_WIRED = False
HUSKY_SCREEN_WIDTH = 320 # pixels
HUSKY_SCREEN_HEIGHT = 240 # pixels
HUSKY_SCREEN_CENTER_X = HUSKY_SCREEN_WIDTH / 2
HUSKY_SCREEN_CENTER_Y = HUSKY_SCREEN_HEIGHT / 2
HUSKY_SCREEN_TOLERANCE_X = 10 # pixels


BALL_DIAMETER = 4 # (cm) ball diameter in real world
DISTANCE_50CM = 50 # (cm)
BALL_FRAMESIZE_50CM = 30 # (pixels) width and height of a ball frame at 50 cm distance
BALL_DISTANCE_RATIO = DISTANCE_50CM / BALL_FRAMESIZE_50CM

QR_CODES = [
    {"code":"id300", "cardinal": "East", "husky_learned_id": 1},
    {"code":"id301", "cardinal": "South", "husky_learned_id": 2},
    {"code":"id302", "cardinal": "West", "husky_learned_id": 3},
    {"code":"id303", "cardinal": "North", "husky_learned_id": 4},
    {"code":"id407", "cardinal": "Base", "husky_learned_id": 5}
    ]

class RobotState:
    halted = 1
    idle = 2
    scouting = 3
    trackingBall = 4
    searchingBalls = 5
    searchingHome = 6
    goingHome = 7

class MotionDirection:
    Idle = 1
    Forward = 2
    Backward = 3
    Spinning = 4

class MotionOrientation:
    Left = 1
    Right = 2
    Straight = 3

class Waypoint:
    def __init__(self, distance, angle):
        self.distance = distance # mm
        self.angle = angle # radians

class TrackedObject():
    def __init__(self):
        self.isTracked = False
        self.isLearned = False
        self.reset()
    def setCoordinates(self,x,y,w,h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.lastSeen = input.running_time()
    def reset(self):
        self.setCoordinates(0,0,0,0)
        self.lastSeen = 0
    # Compute coordinates of the TrackedObject relative to the robot position (origin)
    # (TO BE TESTED)
    # we can make it more empirical :
    # - return a +/- 15 degrees to the left or the right and this will be engouh to guide the robot to the target progressively
    # - determine delta_x as a fraction of a lateral range
    def getTargetSide(self):
        if self.x > (HUSKY_SCREEN_CENTER_X + HUSKY_SCREEN_TOLERANCE_X):
            return MotionOrientation.Right
        elif (HUSKY_SCREEN_CENTER_X - HUSKY_SCREEN_TOLERANCE_X) > self.x:
            return MotionOrientation.Left
        else: 
            return MotionOrientation.Straight
    # Compute coordinates of the TrackedObject relative to the robot position (origin)
    # (TO BE TESTED)
    # we can make it more empirical :
    # - return a +/- 15 degrees to the left or the right and this will be engouh to guide the robot to the target progressively
    # - determine delta_x as a fraction of a lateral range
    def getWaypoint(self):
        distance = min(self.w,self.h) * BALL_DISTANCE_RATIO
        delta_x = abs(self.x - HUSKY_SCREEN_CENTER_X)
        delta_y = Math.sqrt(distance**2 - delta_x**2)
        # Angle between y front-axis and the ball projected coordinates(dx,dy)
        angle_rad = Math.atan2(delta_x, delta_y)
        return Waypoint(distance, angle_rad)

def calculateDistance(width, height):
    REFERENCE_SIZE = 100.0    # 100 pixels 
    REFERENCE_DISTANCE = 50.0 # at 50 cms distance
    size = Math.sqrt(width ** 2 + height ** 2)
    return (REFERENCE_SIZE * REFERENCE_DISTANCE) / size

trackedObject = TrackedObject()

def serial_log(msg: str):
    if EXEC_MODE == ExecMode.MakeCode:
        serial.write_line(msg)
    if EXEC_MODE == ExecMode.WiredMode:
        serial.write_line(msg)
    pass

class Robot:
    def __init__(self):
        self.state = RobotState.halted
    def setState(self, state):
        if (self.state != state) :
           self.state = state
           serial_log("Robot State changed : " + str(self.state))

robot = Robot()

def loop_update_sensors():
    global compass_heading
    # Update Compass direction, current speed, deviation, commands coming from Bluetooth, ...
    # TO DO Compute current position with sensor fusion https://github.com/micropython-IMU/micropython-fusion/tree/master
    compass_heading = input.compass_heading()
    mag_x = input.magnetic_force(Dimension.X)
    mag_y = input.magnetic_force(Dimension.Y)
    mag_z = input.magnetic_force(Dimension.Z)
    acc_x = input.acceleration(Dimension.X)
    acc_y = input.acceleration(Dimension.Y)
    acc_z = input.acceleration(Dimension.Z)
    acceleration = Math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
    pass

#huskylens.getBox_S(1, HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK)

def loop_update_vision():
    # Capture a new video frame and analyze it : is there 1 ball, no ball, a Tag, an obstacle or nothing ?
    if not HUSKY_WIRED:
        return
    huskylens.request()
    serial.write_value("Husky Result IDs", huskylens.get_ids())
    #######################################################
    if (hl_mode == protocolAlgorithm.ALGORITHM_OBJECT_TRACKING):
        # Frame (== Block type) appears in screen ?
        trackedObject.isTracked = huskylens.isAppear_s(HUSKYLENSResultType_t.HUSKYLENSResultBlock) 
        if trackedObject.isTracked:
            objectId = 1 # always one single TrackedObject
            trackedObject.isLearned = huskylens.isLearned(objectId) 
            if trackedObject.isLearned :
                serial_log("Tracked Object Found")
                trackedObject.setCoordinates(
                    # Content1 is struct for a Frame
                    # Content2 is struct for an Arrow
                    huskylens.readeBox(objectId, Content1.xCenter),
                    huskylens.readeBox(objectId, Content1.yCenter),
                    huskylens.readeBox(objectId, Content1.width),
                    huskylens.readeBox(objectId, Content1.height)
                    )
            else : 
                trackedObject.reset()
        else: 
            # if (input.running_time() - trackedObject.lastSeen) > OBJECT_LOST_DELAY:
            trackedObject.reset()
            serial_log("Tracked Object Lost")
            # > try reverse motion or look around
    # Multiple objects
    if (hl_mode in (
            protocolAlgorithm.ALGORITHM_TAG_RECOGNITION,
            protocolAlgorithm.ALGORITHM_COLOR_RECOGNITION)):
        # for each frame, Update the relative Position of the QR codes
        nb_frames = huskylens.getBox(HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK)
        for i in range(1,nb_frames+1):
            id = huskylens.readBox_ss(i, Content3.ID)
            x = huskylens.readBox_ss(i, Content3.xCenter)
            y = huskylens.readBox_ss(i, Content3.yCenter)
            w = huskylens.readBox_ss(i, Content3.width)
            h = huskylens.readBox_ss(i, Content3.height)
            if hl_mode == protocolAlgorithm.ALGORITHM_TAG_RECOGNITION:
                serial_log("QR Tag found with id " + str(id))
                #id = qrcode["husky_learned_id"]
            if hl_mode == protocolAlgorithm.ALGORITHM_COLOR_RECOGNITION:
                serial_log("Colored object found with color code " + str(id))
    #######################################################
    # mode = OBJECTCLASSIFICATION
    #huskylens.get_ids()
    #huskylens.readBox_s(Content3.ID)
    # global object_list
    # object_list = [
    #       huskylens.readBox_ss(1, Content3.ID),
    #       huskylens.readBox_ss(2, Content3.ID),
    #       huskylens.readBox_ss(3, Content3.ID),
    #       huskylens.readBox_ss(4, Content3.ID)]
    # basic.show_string("" + str(object_list[0]) + ("" + str(object_list[1])) + ("" + str(object_list[2])) + ("" + str(object_list[3])))
    pass

def loop_compute_waypoint():
    # Based on the Robot State, Goal and Environment
    # Determine the next waypoint to reach, and linear and angular velocities to reach it
    pass

def loop_motor_controller_run():
    # Drive servo and motor with PWM according to updated linear and angular velocities
    # Set the steering servo position to aim to the waypoint
    steering = 0 # to be computed with PID
    if EXEC_MODE == ExecMode.MakeCode:
        servos.P0.set_angle(steering)
    else:
        amaker_motor.servo_position(amaker_motor.Servos.S1, steering)
    # Set the servo throttle power depending on the remaining distance to the waypoint
    pwm = 50 # to be computed
    if EXEC_MODE == ExecMode.MakeCode:
        servos.P1.run(pwm)
    else:
        amaker_motor.servo_speed(amaker_motor.Servos.S3, pwm)

def loop_state_machine():
    # Based on input signals, current position, object recgnition and game instructions
    # determine the next action and next state
    #
    # Condition 1 : get closer to the home before the end of the game
    # (TO DO : consider distance_to_home)
    countdown = GAME_DURATION - runtime_ms*1000
    if (countdown < TIME_TO_GO_HOME):
        if robot.state not in (RobotState.searchingHome,RobotState.goingHome):
            robot.setState(RobotState.searchingHome)
    # Condition 2 : ...
    if EXEC_MODE == ExecMode.LiveMode:
        huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
        UTBBot.new_bot_status(UTBBotCode.BotStatus.IDLE)
    pass
    
def loop_feedback():
    # Provide feedback to exterior using display and radio
    # Display State
    basic.show_number(robot.state)
    # Send status to game server
    if EXEC_MODE == ExecMode.LiveMode:
        UTBBot.emit_status()
        UTBBot.emit_heart_beat()
    pass
def loop_telemetry():
    # Log metrics and update Display
    if HUSKY_WIRED:
        #   huskylens.write_osd("angle diff "+str(steering_angle), 150, 30)
        #   huskylens.write_osd("servo_angle "+str(servo_angle), 150, 60)
        #   huskylens.write_osd("input "+str(steering_value), 150, 90)
        huskylens.write_osd("head "+str(compass_heading), 150, 120)
    if EXEC_MODE in (ExecMode.MakeCode, ExecMode.WiredMode):
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
initialized = False

def init():
    # Boot sequence
    # Initialization of the sensors, variables, display, callbacks
    basic.clear_screen()
    if EXEC_MODE in (ExecMode.WiredMode, ExecMode.LiveMode):
        input.calibrate_compass()
        input.set_accelerometer_range(AcceleratorRange.ONE_G)
    if HUSKY_WIRED:
        huskylens.init_i2c()
        huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    if EXEC_MODE in (ExecMode.MakeCode, ExecMode.WiredMode):
        datalogger.set_column_titles("target_pos_dist", "target_pos_theta", "velocity_speed", "velocity_angle", "compass_heading")
    global initialized
    initialized = True
    serial_log("Initialization completed")

init()

def on_forever():
    global initialized
    global cycles_count
    # Infinite loop (frequency = ?? Hz)
    if not initialized: # https://support.microbit.org/support/solutions/articles/19000053084-forever-runs-before-onstart-finishes
        return
    cycles_count += 1
    serial_log(str(cycles_count))
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
