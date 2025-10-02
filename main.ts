let hl_mode = 0
class ExecMode {
    static MakeCode: number
    private ___MakeCode_is_set: boolean
    private ___MakeCode: number
    get MakeCode(): number {
        return this.___MakeCode_is_set ? this.___MakeCode : ExecMode.MakeCode
    }
    set MakeCode(value: number) {
        this.___MakeCode_is_set = true
        this.___MakeCode = value
    }
    
    static LiveMode: number
    private ___LiveMode_is_set: boolean
    private ___LiveMode: number
    get LiveMode(): number {
        return this.___LiveMode_is_set ? this.___LiveMode : ExecMode.LiveMode
    }
    set LiveMode(value: number) {
        this.___LiveMode_is_set = true
        this.___LiveMode = value
    }
    
    static WiredMode: number
    private ___WiredMode_is_set: boolean
    private ___WiredMode: number
    get WiredMode(): number {
        return this.___WiredMode_is_set ? this.___WiredMode : ExecMode.WiredMode
    }
    set WiredMode(value: number) {
        this.___WiredMode_is_set = true
        this.___WiredMode = value
    }
    
    public static __initExecMode() {
        ExecMode.MakeCode = 1
        //  Offline software development with no attached board
        ExecMode.LiveMode = 2
        //  Free running Robot
        ExecMode.WiredMode = 3
    }
    
}

ExecMode.__initExecMode()

//  Board wired with USB
let EXEC_MODE = ExecMode.MakeCode
let HUSKY_WIRED = false
let HUSKY_SCREEN_WIDTH = 320
//  pixels
let HUSKY_SCREEN_HEIGHT = 240
//  pixels
let HUSKY_SCREEN_CENTER_X = HUSKY_SCREEN_WIDTH / 2
let HUSKY_SCREEN_CENTER_Y = HUSKY_SCREEN_HEIGHT / 2
let HUSKY_SCREEN_TOLERANCE_X = 10
//  pixels
let BALL_DIAMETER = 4
//  (cm) ball diameter in real world
let DISTANCE_50CM = 50
//  (cm)
let BALL_FRAMESIZE_50CM = 30
//  (pixels) width and height of a ball frame at 50 cm distance
let BALL_DISTANCE_RATIO = DISTANCE_50CM / BALL_FRAMESIZE_50CM
class RobotState {
    static Halted: number
    private ___Halted_is_set: boolean
    private ___Halted: number
    get Halted(): number {
        return this.___Halted_is_set ? this.___Halted : RobotState.Halted
    }
    set Halted(value: number) {
        this.___Halted_is_set = true
        this.___Halted = value
    }
    
    static Idle: number
    private ___Idle_is_set: boolean
    private ___Idle: number
    get Idle(): number {
        return this.___Idle_is_set ? this.___Idle : RobotState.Idle
    }
    set Idle(value: number) {
        this.___Idle_is_set = true
        this.___Idle = value
    }
    
    static Scouting: number
    private ___Scouting_is_set: boolean
    private ___Scouting: number
    get Scouting(): number {
        return this.___Scouting_is_set ? this.___Scouting : RobotState.Scouting
    }
    set Scouting(value: number) {
        this.___Scouting_is_set = true
        this.___Scouting = value
    }
    
    static TrackingBall: number
    private ___TrackingBall_is_set: boolean
    private ___TrackingBall: number
    get TrackingBall(): number {
        return this.___TrackingBall_is_set ? this.___TrackingBall : RobotState.TrackingBall
    }
    set TrackingBall(value: number) {
        this.___TrackingBall_is_set = true
        this.___TrackingBall = value
    }
    
    static SearchingBalls: number
    private ___SearchingBalls_is_set: boolean
    private ___SearchingBalls: number
    get SearchingBalls(): number {
        return this.___SearchingBalls_is_set ? this.___SearchingBalls : RobotState.SearchingBalls
    }
    set SearchingBalls(value: number) {
        this.___SearchingBalls_is_set = true
        this.___SearchingBalls = value
    }
    
    static SearchingHome: number
    private ___SearchingHome_is_set: boolean
    private ___SearchingHome: number
    get SearchingHome(): number {
        return this.___SearchingHome_is_set ? this.___SearchingHome : RobotState.SearchingHome
    }
    set SearchingHome(value: number) {
        this.___SearchingHome_is_set = true
        this.___SearchingHome = value
    }
    
    static GoingHome: number
    private ___GoingHome_is_set: boolean
    private ___GoingHome: number
    get GoingHome(): number {
        return this.___GoingHome_is_set ? this.___GoingHome : RobotState.GoingHome
    }
    set GoingHome(value: number) {
        this.___GoingHome_is_set = true
        this.___GoingHome = value
    }
    
    public static __initRobotState() {
        RobotState.Halted = 1
        RobotState.Idle = 2
        RobotState.Scouting = 3
        RobotState.TrackingBall = 4
        RobotState.SearchingBalls = 5
        RobotState.SearchingHome = 6
        RobotState.GoingHome = 7
    }
    
}

RobotState.__initRobotState()

class MotionDirection {
    static Idle: number
    private ___Idle_is_set: boolean
    private ___Idle: number
    get Idle(): number {
        return this.___Idle_is_set ? this.___Idle : MotionDirection.Idle
    }
    set Idle(value: number) {
        this.___Idle_is_set = true
        this.___Idle = value
    }
    
    static Forward: number
    private ___Forward_is_set: boolean
    private ___Forward: number
    get Forward(): number {
        return this.___Forward_is_set ? this.___Forward : MotionDirection.Forward
    }
    set Forward(value: number) {
        this.___Forward_is_set = true
        this.___Forward = value
    }
    
    static Backward: number
    private ___Backward_is_set: boolean
    private ___Backward: number
    get Backward(): number {
        return this.___Backward_is_set ? this.___Backward : MotionDirection.Backward
    }
    set Backward(value: number) {
        this.___Backward_is_set = true
        this.___Backward = value
    }
    
    static Spinning: number
    private ___Spinning_is_set: boolean
    private ___Spinning: number
    get Spinning(): number {
        return this.___Spinning_is_set ? this.___Spinning : MotionDirection.Spinning
    }
    set Spinning(value: number) {
        this.___Spinning_is_set = true
        this.___Spinning = value
    }
    
    public static __initMotionDirection() {
        MotionDirection.Idle = 1
        MotionDirection.Forward = 2
        MotionDirection.Backward = 3
        MotionDirection.Spinning = 4
    }
    
}

MotionDirection.__initMotionDirection()

class MotionOrientation {
    static Left: number
    private ___Left_is_set: boolean
    private ___Left: number
    get Left(): number {
        return this.___Left_is_set ? this.___Left : MotionOrientation.Left
    }
    set Left(value: number) {
        this.___Left_is_set = true
        this.___Left = value
    }
    
    static Right: number
    private ___Right_is_set: boolean
    private ___Right: number
    get Right(): number {
        return this.___Right_is_set ? this.___Right : MotionOrientation.Right
    }
    set Right(value: number) {
        this.___Right_is_set = true
        this.___Right = value
    }
    
    static Straight: number
    private ___Straight_is_set: boolean
    private ___Straight: number
    get Straight(): number {
        return this.___Straight_is_set ? this.___Straight : MotionOrientation.Straight
    }
    set Straight(value: number) {
        this.___Straight_is_set = true
        this.___Straight = value
    }
    
    public static __initMotionOrientation() {
        MotionOrientation.Left = 1
        MotionOrientation.Right = 2
        MotionOrientation.Straight = 3
    }
    
}

MotionOrientation.__initMotionOrientation()

class Waypoint {
    distance: number
    angle: number
    constructor(distance: number, angle: number) {
        this.distance = distance
        //  mm
        this.angle = angle
    }
    
}

//  radians
class TrackedObject {
    isTracked: boolean
    isLearned: boolean
    x: number
    y: number
    w: number
    h: number
    constructor() {
        this.isTracked = false
        this.isLearned = false
        this.reset()
    }
    
    public setCoordinates(x: number, y: number, w: number, h: number) {
        this.x = x
        this.y = y
        this.w = w
        this.h = h
    }
    
    public reset() {
        this.setCoordinates(0, 0, 0, 0)
    }
    
    //  Compute coordinates of the TrackedObject relative to the robot position (origin)
    //  (TO BE TESTED)
    //  we can make it more empirical :
    //  - return a +/- 15 degrees to the left or the right and this will be engouh to guide the robot to the target progressively
    //  - determine delta_x as a fraction of a lateral range
    public getTargetSide(): number {
        if (this.x > HUSKY_SCREEN_CENTER_X + HUSKY_SCREEN_TOLERANCE_X) {
            return MotionOrientation.Right
        } else if (HUSKY_SCREEN_CENTER_X - HUSKY_SCREEN_TOLERANCE_X > this.x) {
            return MotionOrientation.Left
        } else {
            return MotionOrientation.Straight
        }
        
    }
    
    //  Compute coordinates of the TrackedObject relative to the robot position (origin)
    //  (TO BE TESTED)
    //  we can make it more empirical :
    //  - return a +/- 15 degrees to the left or the right and this will be engouh to guide the robot to the target progressively
    //  - determine delta_x as a fraction of a lateral range
    public getWaypoint(): Waypoint {
        let distance = Math.min(this.w, this.h) * BALL_DISTANCE_RATIO
        let delta_x = Math.abs(this.x - HUSKY_SCREEN_CENTER_X)
        let delta_y = Math.sqrt(distance ** 2 - delta_x ** 2)
        //  Angle between y front-axis and the ball projected coordinates(dx,dy)
        let angle_rad = Math.atan2(delta_x, delta_y)
        return new Waypoint(distance, angle_rad)
    }
    
    public getLocation() {
        let mapx = Math.map(this.x, -160, 160, 0, 80)
        let delta_x = 80 - this.x
        let delta_y = Math.map(this.y, 0, 210, 95, 160)
    }
    
}

let trackedObject = new TrackedObject()
function serial_log(msg: string) {
    if (EXEC_MODE == ExecMode.MakeCode) {
        serial.writeLine(msg)
    }
    
    if (EXEC_MODE == ExecMode.WiredMode) {
        serial.writeLine(msg)
    }
    
    
}

function loop_update_sensors() {
    
    //  Update Compass direction, current speed, deviation, commands coming from Bluetooth, ...
    //  Compute current position
    compass_heading = input.compassHeading()
    
}

// huskylens.getBox_S(1, HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK)
function loop_update_vision() {
    let objectId: number;
    //  Capture a new video frame and analyze it : is there 1 ball, no ball, a Tag, an obstacle or nothing ?
    if (!HUSKY_WIRED) {
        return
    }
    
    huskylens.request()
    serial.writeValue("Husky Result IDs", huskylens.getIds())
    // ######################################################
    if (hl_mode == protocolAlgorithm.ALGORITHM_OBJECT_TRACKING) {
        //  Frame (== Block type) appears in screen ?
        trackedObject.isTracked = huskylens.isAppear_s(HUSKYLENSResultType_t.HUSKYLENSResultBlock)
        if (trackedObject.isTracked) {
            objectId = 1
            //  always one single TrackedObject
            trackedObject.isLearned = huskylens.isLearned(objectId)
            if (trackedObject.isLearned) {
                serial_log("Tracked Object Found")
                trackedObject.setCoordinates(huskylens.readeBox(objectId, Content1.xCenter), huskylens.readeBox(objectId, Content1.yCenter), huskylens.readeBox(objectId, Content1.width), huskylens.readeBox(objectId, Content1.height))
            } else {
                //  Content1 is struct for a Frame
                //  Content2 is struct for an Arrow
                trackedObject.reset()
            }
            
        } else {
            trackedObject.reset()
            serial_log("Tracked Object Lost")
        }
        
    }
    
    //  > try reverse motion or look around
    // ######################################################
    //  mode = OBJECTCLASSIFICATION
    // huskylens.get_ids()
    // huskylens.readBox_s(Content3.ID)
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
    if (EXEC_MODE == ExecMode.MakeCode) {
        servos.P0.setAngle(steering)
    } else {
        amaker_motor.servoPosition(amaker_motor.Servos.S1, steering)
    }
    
    //  Set the servo throttle power depending on the remainning distance to the waypoint
    let pwm = 50
    //  to be computed
    if (EXEC_MODE == ExecMode.MakeCode) {
        servos.P1.run(pwm)
    } else {
        amaker_motor.servoSpeed(amaker_motor.Servos.S3, pwm)
    }
    
}

function loop_state_machine() {
    //  Based on input signals, current position, object recgnition and game instructions
    //  determine the next action and next state
    if (EXEC_MODE == ExecMode.LiveMode) {
        huskylens.initMode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
        UTBBot.newBotStatus(UTBBotCode.BotStatus.Idle)
    }
    
    
}

function loop_feedback() {
    //  Provide feedback to exterior using display and radio
    basic.showArrow(ArrowNames.North)
    if (EXEC_MODE == ExecMode.LiveMode) {
        UTBBot.emitStatus()
        UTBBot.emitHeartBeat()
    }
    
    
}

function loop_telemetry() {
    //  Log metrics and update Display
    if (HUSKY_WIRED) {
        //    huskylens.write_osd("angle diff "+str(steering_angle), 150, 30)
        //    huskylens.write_osd("servo_angle "+str(servo_angle), 150, 60)
        //    huskylens.write_osd("input "+str(steering_value), 150, 90)
        huskylens.writeOSD("head " + ("" + compass_heading), 150, 120)
    }
    
    if ([ExecMode.MakeCode, ExecMode.WiredMode].indexOf(EXEC_MODE) >= 0) {
        datalogger.log(datalogger.createCV("compass_heading", compass_heading))
    }
    
}

//        datalogger.create_cv("target_pos_dist", target_pos_dist)
//        ,datalogger.create_cv("target_pos_theta", target_pos_theta)
//        ,datalogger.create_cv("velocity_speed", velocity_speed)
//        ,datalogger.create_cv("velocity_angle ", velocity_angle )
let cycles_count = 0
let compass_heading = 0
let avg_loop_rate = 0
let runtime_ms = 0
let initialized = false
function init() {
    //  Boot sequence
    //  Initialization of the sensors, variables, display, callbacks
    basic.clearScreen()
    if ([ExecMode.WiredMode, ExecMode.LiveMode].indexOf(EXEC_MODE) >= 0) {
        input.calibrateCompass()
        input.setAccelerometerRange(AcceleratorRange.OneG)
    }
    
    if (HUSKY_WIRED) {
        huskylens.initI2c()
        huskylens.initMode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    }
    
    if ([ExecMode.MakeCode, ExecMode.WiredMode].indexOf(EXEC_MODE) >= 0) {
        datalogger.setColumnTitles("target_pos_dist", "target_pos_theta", "velocity_speed", "velocity_angle", "compass_heading")
    }
    
    
    initialized = true
    serial_log("Initialization completed")
}

init()
//  best effort loop
basic.forever(function on_forever() {
    
    
    //  Infinite loop (frequency = ?? Hz)
    if (!initialized) {
        //  https://support.microbit.org/support/solutions/articles/19000053084-forever-runs-before-onstart-finishes
        return
    }
    
    cycles_count += 1
    serial_log("" + cycles_count)
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
