hl_mode = 0

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

class RobotState:
    Halted = 1
    Idle = 2
    Scouting = 3
    TrackingBall = 4
    SearchingBalls = 5
    SearchingHome = 6
    GoingHome = 7

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
    def reset(self):
        self.setCoordinates(0,0,0,0)
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
    def getLocation(self):
        mapx = Math.map(self.x, -160, 160, 0, 80)
        delta_x = 80 - self.x
        delta_y = Math.map(self.y, 0, 210, 95, 160)

trackedObject = TrackedObject()

def serial_log(msg: str):
    if EXEC_MODE == ExecMode.MakeCode:
        serial.write_line(msg)
    if EXEC_MODE == ExecMode.WiredMode:
        serial.write_line(msg)
    pass
def loop_update_sensors():
    global compass_heading
    # Update Compass direction, current speed, deviation, commands coming from Bluetooth, ...
    # Compute current position
    compass_heading = input.compass_heading()
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
            trackedObject.reset()
            serial_log("Tracked Object Lost")
            # > try reverse motion or look around
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
    if EXEC_MODE == ExecMode.MakeCode:
        servos.P0.set_angle(steering)
    else:
        amaker_motor.servo_position(amaker_motor.Servos.S1, steering)
    # Set the servo throttle power depending on the remainning distance to the waypoint
    pwm = 50 # to be computed
    if EXEC_MODE == ExecMode.MakeCode:
        servos.P1.run(pwm)
    else:
        amaker_motor.servo_speed(amaker_motor.Servos.S3, pwm)

def loop_state_machine():
    # Based on input signals, current position, object recgnition and game instructions
    # determine the next action and next state
    if EXEC_MODE == ExecMode.LiveMode:
        huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
        UTBBot.new_bot_status(UTBBotCode.BotStatus.IDLE)
    pass
    
def loop_feedback():
    # Provide feedback to exterior using display and radio
    basic.show_arrow(ArrowNames.NORTH)
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
