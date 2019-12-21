package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    //SERVO OBJECTS
    Servo latchServo1, latchServo2;
    Servo grabberServo;
    Servo hungryHippoServo;
    Servo armServo1, armServo2;
    Servo servoLink1, servoLink2;

    //MOTOR OBJECTS
    DcMotor lift1, lift2;
    DcMotor intake1, intake2;

    DriveController driveController;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;

    //SENSORS
    DistanceSensor frontRangeSensor;
    DistanceSensor backRangeSensor;

    final DcMotor.RunMode DEFAULT_RUN_MODE;
    final boolean IS_AUTO;
    int targetPosLift;
    double LIFT_TICKS_PER_MS = 50.0/30.0;

    boolean wasAtZero = true; //assume the lift always starts unpowered?
    double lastTimeLift;
    //boolean IMUReversed = false;

    // only 8 levels are valid
    int[] encoderTicksAtLiftPositions = new int[8];
    int liftPosition = 0;
    boolean wasLastPositive = false;

    //lift constants
    //final double barLength = 76.8; //units: cm
    //final int ticksAtStraight = 0;
//    final double liftStartHeight = 4.60375, barLength = 38.4, blockHeight = 10.16;
    // adjusted the lowest height to 4.3cm (measured on the robot
    final double liftStartHeight = 4.3, barLength = 38.4, blockHeight = 10.16;
    final double startAngle = Math.asin(liftStartHeight / barLength);
    final double ticksPerEntireRotation = 28*13.7*302.0/14;
    final double twoPi = 2 * Math.PI;

    public Robot (OpMode opMode, boolean isAuto) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        driveController = new DriveController(this);
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");

        IS_AUTO = isAuto;
        DEFAULT_RUN_MODE = isAuto ? AUTO_RUN_MODE : TELEOP_RUN_MODE;

        //the cooper thing
        latchServo1 = hardwareMap.servo.get("latchServo1");
        setupServo(latchServo1);
        latchServo2 = hardwareMap.servo.get("latchServo2");
        setupServo(latchServo2);

        //the cyrus thing
        hungryHippoServo = hardwareMap.servo.get("hungryHippoServo");
        setupServo(hungryHippoServo);

        //the kevin things
        //deprecated
        armServo1 = hardwareMap.servo.get("armServo1");
        setupServo(armServo1);
        armServo2 = hardwareMap.servo.get("armServo2");
        setupServo(armServo2);

        //new
        servoLink1 = hardwareMap.servo.get("servoLink1");
        setupServo(servoLink1);
        servoLink2 = hardwareMap.servo.get("servoLink2");
        setupServo(servoLink2);

        grabberServo = hardwareMap.servo.get("grabberServo");
        setupServo(grabberServo);

        setupLift();

        intake1 = hardwareMap.dcMotor.get("intakeMotor1");
        setupMotor(intake1, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        intake2 = hardwareMap.dcMotor.get("intakeMotor2");
        setupMotor(intake2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);

        frontRangeSensor = hardwareMap.get(DistanceSensor.class, "frontRangeSensor");
        backRangeSensor = hardwareMap.get(DistanceSensor.class, "backRangeSensor");
    }

    public void initIMU () {
        //this.IMUReversed = reversed;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    //public void initIMU () { initIMU(IMUReversed); }

    public Angle getRobotHeading () {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("Robot Heading", heading);
//        if (IMUReversed) {
//            return new Angle(heading-180, Angle.AngleType.NEG_180_TO_180_HEADING);
//        }
        return new Angle(heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    //SETUP METHODS
    public void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DEFAULT_RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //takes RunMode parameter
    public void setupMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode RUN_MODE) {
        motor.setDirection(direction);
        motor.setMode(RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setupServo(Servo servo) {
        servo.setDirection(DEFAULT_SERVO_DIRECTION);
    }

    //ANGLE METHODS
//    public Orientation getAngleOrientation() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); }
//    public double getCurrentAngleZ() { return getAngleOrientation().firstAngle; }
//    public double getCurrentAngleY() { return getAngleOrientation().secondAngle; }
//    public double getCurrentAngleX() { return getAngleOrientation().thirdAngle; }
//    public static double getZAngle(Orientation o) { return o.firstAngle; }
//    public static double getYAngle(Orientation o) { return o.secondAngle; }
//    public static double getXAngle(Orientation o) { return o.thirdAngle; }
//    public static double wrapAngle(double angle) { return angle < 0 ? angle % (2 * Math.PI) + 2 * Math.PI : angle % (2 * Math.PI); }

    //SERVOS
    public void moveServo(Servo servo, double pos) { servo.setPosition(pos); }

    public void unlatch() {
        moveServo(latchServo1, UNLATCHED_POSITION_1);
        moveServo(latchServo2, UNLATCHED_POSITION_2);
    }

    public void latch() {
        moveServo(latchServo2, LATCHED_POSITION_2);
        moveServo(latchServo1, LATCHED_POSITION_1);
    }

    public void closeGrabber() {
        moveServo(grabberServo, GRABBER_CLOSE_POSITION);
    }
    public void openGrabber() {
        moveServo(grabberServo, GRABBER_OPEN_POSITION);
    }

    public void hungryHippoExtend(){
        moveServo(hungryHippoServo, HUNGRY_HIPPO_EXTEND_POSITION);
    }
    public void hungryHippoRetract(){
        moveServo(hungryHippoServo, HUNGRY_HIPPO_RETRACT_POSITION);
    }


    //MOTORS
    public void moveMotor(DcMotor motor, double power) { motor.setPower(power); }

    public void setArmPower(double power){
        if (power > 0.1) {
            armServo1.setPosition(1);
            armServo2.setPosition(0);
        } else if (power < -0.1) {
            armServo1.setPosition(0);
            armServo2.setPosition(1);
        } else {
            armServo1.setPosition(0.5);
            armServo2.setPosition(0.5);
        }
//        moveServo(armServo1, servoPowerCalc(power));
//        moveServo(armServo2, servoPowerCalc(-power));
//        telemetry.addData( "outtake 1: ", servoPowerCalc(power));
//        telemetry.addData("outtake 2: ", servoPowerCalc(-power));
//        telemetry.update();
    }

    public double determineBlockPlacementTicks(int block) {
        return ((Math.asin((liftStartHeight + (blockHeight / 2) * block) / barLength) - startAngle) / (twoPi)) * ticksPerEntireRotation;
    }

    public double scaleLiftRate(double initalRate, double deltaTime) {
        //scaledRate = RobotUtil.scaleVal(initalRate, 0, 1, )

        if (initalRate < 0) {
            // go slower on the way down (or not)
            return initalRate * 75;
        } else {
            return initalRate * 50;
        }
    }

    public void setupLift() {
        //the obvious things
        lift1 = hardwareMap.dcMotor.get("liftMotorLeft");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(lift1.getCurrentPosition());
        setupMotor(lift1, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_TO_POSITION);

        lift2 = hardwareMap.dcMotor.get("liftMotorRight");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setTargetPosition(lift2.getCurrentPosition());
        setupMotor(lift2, DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < encoderTicksAtLiftPositions.length; i++) {
            encoderTicksAtLiftPositions[i] = (int)determineBlockPlacementTicks(i);
        }
        // limit the top position to the physical constraints of the lift
        encoderTicksAtLiftPositions[encoderTicksAtLiftPositions.length - 1] = (int)(((Math.asin(73.10 /*max height*// 2 / barLength) - startAngle) / (twoPi)) * ticksPerEntireRotation);
    }

    public int getLiftBlockPositionToGo(int position, boolean goingUp) {
        if (goingUp) {
            for (int i = 0; i < encoderTicksAtLiftPositions.length; i++) {
                if (position < encoderTicksAtLiftPositions[i]) {
                    return encoderTicksAtLiftPositions[i];
                }
            }
            return encoderTicksAtLiftPositions[encoderTicksAtLiftPositions.length - 1];
        } else {
            for (int i = encoderTicksAtLiftPositions.length - 1; i >= 0; i--) {
                if (position > encoderTicksAtLiftPositions[i]) {
                    return encoderTicksAtLiftPositions[i];
                }
            }
            return encoderTicksAtLiftPositions[0];
        }
    }

    public void moveLift(double moveRate) {
//        double deltaTime = System.currentTimeMillis() - lastTimeLift;
//        wasAtZero = true;
//        int lift1CurrentPos = lift1.getCurrentPosition();
//        int lift2CurrentPos = lift2.getCurrentPosition();
//        //int position = (lift1CurrentPos + lift2CurrentPos)/2; //integer division, i know (i think it's fine)
//        int position = lift1CurrentPos; //integer division, i know (i think it's fine)
//
//        if (Math.abs(moveRate) < 0.1) {
//            moveRate = 0;
//        } else {
//            wasAtZero = false;
//        }
//
//        //double scaledMoveRate = scaleLiftRate(moveRate, deltaTime);
//        double scaledMoveRate = moveRate * LIFT_TICKS_PER_MS * deltaTime;
//        //double scaledMoveRate = moveRate * 60;
//        targetPosLift += scaledMoveRate;
//        if (moveRate == 0) {
//            if (!wasAtZero) {
//                wasAtZero = true;
//                //position = (lift1CurrentPos + lift2CurrentPos)/2;
//                position = lift1CurrentPos;
//            }
//            targetPosLift = position;
//        }
//
//        if (lift1.getMode() != DcMotor.RunMode.RUN_TO_POSITION) lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if (lift2.getMode() != DcMotor.RunMode.RUN_TO_POSITION) lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift1.setTargetPosition(targetPosLift);
//        lift1.setPower(1);
//        lift2.setTargetPosition(targetPosLift);
//        lift2.setPower(1);
//
//        lastTimeLift = System.currentTimeMillis();
//
//        telemetry.addData("Target Position for Lift", targetPosLift);
//        telemetry.addData("Current Lift Power", scaledMoveRate);
//        telemetry.addData("Loop time", deltaTime);
//        telemetry.addData("Lift ticks/ms", LIFT_TICKS_PER_MS);

        double deltaTime = System.currentTimeMillis() - lastTimeLift;
        // don't reset static varuable wasAtZero every loop
        //wasAtZero = true;

        int lift1CurrentPos = lift1.getCurrentPosition();
        int lift2CurrentPos = lift2.getCurrentPosition();
        //int position = (lift1CurrentPos + lift2CurrentPos)/2; //integer division, i know (i think it's fine)
        int position = lift1CurrentPos; //integer division, i know (i think it's fine)

        if (Math.abs(moveRate) < 0.1) {
            moveRate = 0;
        } else {
            wasAtZero = false;
        }

        //double scaledMoveRate = scaleLiftRate(moveRate, deltaTime);
        double scaledMoveRate = moveRate * LIFT_TICKS_PER_MS * deltaTime;
        // if (scaledMoveRate < 0) scaledMoveRate *=2; // lift will go down faster than up
        //double scaledMoveRate = moveRate * 60;
        if (moveRate == 0) {
            if (!wasAtZero) {
                wasAtZero = true;
                //position = (lift1CurrentPos + lift2CurrentPos)/2;
                position = getLiftBlockPositionToGo(lift1CurrentPos, wasLastPositive);

                // hold on to the last integral block level as the target position
                targetPosLift = position;

            }
        } else {
            // only update the target position if the move rate is non-zero
            targetPosLift += scaledMoveRate;
            // limit check
            if (targetPosLift < 0) targetPosLift = 0;
            // what is the maximum limit
        }

        if ((lift1CurrentPos < encoderTicksAtLiftPositions[1] * 0.25) &&
                (targetPosLift < encoderTicksAtLiftPositions[1] * 0.25)){ // lift is settling to the ground state so turn off power
            // this height should be ~1 inch above the lowest height so the lift should still be below the 14" bars
            lift1.setPower(0);
            lift2.setPower(0);
        } else { // otherwise use full power to move to and hold the elevated position
            if (lift1.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (lift2.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift1.setTargetPosition(targetPosLift);
            lift2.setTargetPosition(targetPosLift);
            if (targetPosLift ==0) { // use less power when going down
                lift1.setPower(0.5);
                lift2.setPower(0.5);
            } else {
                lift1.setPower(1);
                lift2.setPower(1);
            }
        }

        lastTimeLift = System.currentTimeMillis();

        wasLastPositive = moveRate > 0;

//        telemetry.addData("lift level 0", encoderTicksAtLiftPositions[0] );
//        telemetry.addData("lift level 1", encoderTicksAtLiftPositions[1] );
//        telemetry.addData("lift level 2", encoderTicksAtLiftPositions[2] );
//        telemetry.addData("lift level 3", encoderTicksAtLiftPositions[3] );
//        telemetry.addData("lift level 4", encoderTicksAtLiftPositions[4] );
//        telemetry.addData("lift level 5", encoderTicksAtLiftPositions[5] );
//        telemetry.addData("lift level 6", encoderTicksAtLiftPositions[6] );
//        telemetry.addData("lift level 7", encoderTicksAtLiftPositions[7] );

        telemetry.addData("Target Position for Lift", targetPosLift);
        telemetry.addData("Current Lift Power", scaledMoveRate);
        telemetry.addData("Loop time", deltaTime);
        telemetry.addData("Lift ticks/ms", LIFT_TICKS_PER_MS);
    }

    public void moveIntake(IntakeState state) {
        //Defaults to being fast
        moveIntake(state, IntakeSpeed.FAST);
    }

    public void moveIntakeUntilStalled(IntakeState state, IntakeSpeed speed, long timeout, LinearOpMode linearOpMode) {
        moveIntake(state, speed);
        long startTime = System.currentTimeMillis();
        double previousEncoder = intakeEncoderValue();
        double currentEncoder = previousEncoder + Constants.MIN_ENCODER_DIFFERENCE_INTAKE + 1;
        while (currentEncoder - previousEncoder > Constants.MIN_ENCODER_DIFFERENCE_INTAKE && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive()) {
            telemetry.addData("Encoder change", currentEncoder - previousEncoder);
            telemetry.update();
            wait(200, linearOpMode);
            previousEncoder = currentEncoder;
            currentEncoder = intakeEncoderValue();
        }
    }

    public double intakeEncoderValue() {
        return (intake1.getCurrentPosition() + intake2.getCurrentPosition()) / 2.0;
    }

    public void moveIntake(IntakeState state, IntakeSpeed speed) {
        if (state == IntakeState.INTAKE) {
            if (speed == IntakeSpeed.FAST) {
                intake1.setPower(INTAKE_POWER_FAST);
                intake2.setPower(INTAKE_POWER_FAST);
            } else {
                intake1.setPower(INTAKE_POWER_SLOW);
                intake2.setPower(INTAKE_POWER_SLOW);
            }
        } else if (state == IntakeState.OUTTAKE) {
            if (speed == IntakeSpeed.FAST) {
                intake1.setPower(OUTTAKE_POWER_FAST);
                intake2.setPower(OUTTAKE_POWER_FAST);
            } else {
                intake1.setPower(OUTTAKE_POWER_SLOW);
                intake2.setPower(OUTTAKE_POWER_SLOW);
            }
        } else {
            intake1.setPower(STOP_POWER);
            intake2.setPower(STOP_POWER);
        }
    }

    public void moveSingleIntakeRoller(boolean roller1) {
        (roller1 ? intake1 : intake2).setPower(INTAKE_POWER_SLOW);
    }

    public double servoPowerCalc(double p){
        return (p / 2.0) + .5;
    }

    public void wait (int millis, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && linearOpMode.opModeIsActive()) {}
    }

    public double getRange (boolean frontSensor) {
        if (frontSensor) return frontRangeSensor.getDistance(DistanceUnit.CM);
        else return backRangeSensor.getDistance(DistanceUnit.CM);
    }

    public void resetLiftEncoders () {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //GRABBER LINKAGE

    //adjust depending on the refresh rate
    private final double k = 1;

    // th -> link one servo angle
    // ph -> link two servo angle
    // xin -> x-joystick magnitude
    // yin -> y-joystick magnitude
    // state -> false is Type 1 passed state, true is neutral pentagon

    public void moveOuttake(double th, double ph, int xin, int yin, boolean state){ //this method should be called in the main loop
        double x = getX(th, ph);
        double y = getY(th, ph);
        //check right bound
        if(th < 0.471 && xin > 0)
            xin = 0;
        //check current linkage state
        if((ph > th) == state)
            flip(th, ph);
        //check left bound
        if(Math.atan2(y, x + 60) + Math.acos(Math.hypot(x + 60, y)/240) > 1.099 && xin < 0)
            xin = 0;
        //check lower bound
        if(y <= 0 && yin < 0)
            yin = 0;
        //check upper bound
        if((Math.hypot(x-60,y) >= 115 || Math.hypot(x+60, y) >= 115) && yin > 0)
            yin = 0;
        int stateSign = state ? 1 : -1;
        //compute partial derivatives
        double dthdx = -(y/(3600 - 120*x + x*x + y*y)) + stateSign * (-60 + x)/Math.sqrt(-((-54000 - 120*x + x*x + y*y)*(3600 - 120*x + x*x + y*y)));
        double dthdy = (-60 + x)/(3600 - 120*x + x*x + y*y) + stateSign * y/Math.sqrt(-((-54000 - 120*x + x*x + y*y)*(3600 - 120*x + x*x + y*y)));
        double dphdx = -(y/(3600 - 120*x + x*x + y*y)) - stateSign * (-60 + x)/Math.sqrt(-((-54000 - 120*x + x*x + y*y)*(3600 - 120*x + x*x + y*y)));
        double dphdy = (-60 + x)/(3600 - 120*x + x*x + y*y) - stateSign * y/Math.sqrt(-((-54000 - 120*x + x*x + y*y)*(3600 - 120*x + x*x + y*y)));
        //required velocity of each servo
        double vth = Math.hypot(dthdx * xin, dthdy * yin);
        double vph = Math.hypot(dphdx * xin, dphdy * yin);
        //increments servo in accordance with velocity
        servoLink1.setPosition(th + k * vth);
        servoLink2.setPosition(ph + k * vph);
    }

    //flips Type 1 singularity
    private void flip(double th, double ph){
        servoLink1.setPosition(ph);
        servoLink2.setPosition(th);
    }

    //automatically deploys the outtake
    public boolean deployOuttake(double th, double ph){ //this should also be called repeatedly in the main loop, returns false when complete
        int targetPos = 200;
        double x = getX(th, ph);
        double y = getY(th, ph);
        if(Math.abs(targetPos - x) <= 10){
            if(ph > th)
                flip(th, ph);
            else return false;
        }
        moveOuttake(th, ph, 1023, 0, false);
        return true;
    }

    //automatically returns the outtake
    public boolean returnOuttake(double th, double ph){ //this should also be called repeatedly in the main loop, returns false when complete
        int targetPos = -100;
        double x = getX(th, ph);
        double y = getY(th, ph);
        if(th > ph)
            flip(ph, th);
        if(Math.abs(targetPos - x) <= 10){
            return false;
        }
        moveOuttake(th, ph, -1023, 0, false);
        return true;
    }

    public double getX(double th, double ph){
        return 60 + 120 * Math.cos(th) + 120 * Math.cos(ph);
    }

    public double getY(double th, double ph){
        return 120 * Math.sin(th) + 120 * Math.sin(ph);
    }

}

class Constants {
    final static DcMotor.RunMode AUTO_RUN_MODE = DcMotor.RunMode.RUN_USING_ENCODER, TELEOP_RUN_MODE = DcMotor.RunMode.RUN_USING_ENCODER;
    final static Servo.Direction DEFAULT_SERVO_DIRECTION = Servo.Direction.FORWARD;

    //TODO Define these constants
    final static double WHEEL_DIAMETER = 0, TICKS_PER_ROTATION = 0;
    final static double INCHES_PER_ROTATION = Math.PI * WHEEL_DIAMETER;
    final static double TICKS_PER_INCH = TICKS_PER_ROTATION / INCHES_PER_ROTATION;
    final static double DEGREES_THRESHOLD = 2;

    //TODO FINISH THESE CONSTANTS
    final static double LATCHED_POSITION_1 = 0, UNLATCHED_POSITION_1 = 1, LATCHED_POSITION_2 = 1, UNLATCHED_POSITION_2 = 0;
    final static double HUNGRY_HIPPO_RETRACT_POSITION = 1, HUNGRY_HIPPO_EXTEND_POSITION = 0;
    final static double GRABBER_OPEN_POSITION = 1, GRABBER_CLOSE_POSITION = 0;

    enum IntakeState { INTAKE, OUTTAKE, STOP }

    final static double INTAKE_POWER_FAST = 0.7, OUTTAKE_POWER_FAST = -INTAKE_POWER_FAST,
            INTAKE_POWER_SLOW = 0.45, OUTTAKE_POWER_SLOW = -INTAKE_POWER_SLOW, STOP_POWER = 0; //.7 too fast

    enum IntakeSpeed { FAST, SLOW, STOPPED }

    enum LatchSide { LEFT, RIGHT }

    final static int MIN_ENCODER_DIFFERENCE_INTAKE = 1;
}