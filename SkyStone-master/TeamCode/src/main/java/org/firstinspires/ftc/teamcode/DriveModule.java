package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.revextensions2.ExpansionHubMotor;

public class DriveModule {
    Robot robot;
    public final ModuleSide moduleSide;
    boolean debuggingMode;

    DataLogger dataLogger;

    //module specific drive motors
    ExpansionHubMotor motor1; //top motor
    ExpansionHubMotor motor2; //bottom motor

    double lastM1Encoder;
    double lastM2Encoder;

    public final Vector2d positionVector; //position of module relative to robot COM (center of mass)

    //used for logic that allows robot to rotate modules as little as possible
    public boolean takingShortestPath = false;
    public boolean reversed = false;

    enum RotateModuleMode {
        DO_NOT_ROTATE_MODULES, ROTATE_MODULES
    }

    //important note about naming conventions below:
    // a MODULE rev is when the orientation of the module changes by 360 degrees
    // a WHEEL rev is when the wheel drives a distance equal to its circumference

    public final double TICKS_PER_MODULE_REV = 28 * (double)(60)/14 * (double)(48)/15 * (double)(82)/22; //ticks per MODULE revolution
    public final double DEGREES_PER_TICK = 360/TICKS_PER_MODULE_REV;

    public final double TICKS_PER_WHEEL_REV = 28 * (double)(60)/14 * (double)(48)/15 * (double)(82)/22 * (double)(14)/60; //ticks per WHEEL revolution

    public final double CM_WHEEL_DIAMETER = 3 * 2.54;
    public final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public final double CM_PER_TICK = CM_PER_WHEEL_REV/TICKS_PER_WHEEL_REV;

    //used for scaling pivot component (see getPivotComponent() method)
    public final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 25; //degrees

    //if module is within this number of degrees from its target orientation, no pivot power will be applied
    public final double ALLOWED_MODULE_ORIENTATION_ERROR = 5;

    public double positionChange; //used for position tracking

    public DriveModule(Robot robot, ModuleSide moduleSide, boolean debuggingMode) {
        this.robot = robot;
        this.moduleSide = moduleSide;
        this.debuggingMode = debuggingMode;
        if (moduleSide == ModuleSide.RIGHT) {
            motor1 = (ExpansionHubMotor) robot.hardwareMap.dcMotor.get("rightTopMotor");
            motor2 = (ExpansionHubMotor) robot.hardwareMap.dcMotor.get("rightBottomMotor");
            positionVector = new Vector2d((double)18/2, 0); //points from robot center to right module
        } else {
            motor1 = (ExpansionHubMotor) robot.hardwareMap.dcMotor.get("leftTopMotor");
            motor2 = (ExpansionHubMotor) robot.hardwareMap.dcMotor.get("leftBottomMotor");
            positionVector = new Vector2d((double)-18/2, 0); //points from robot center to left module
        }

        //set run mode to NOT use encoders for velocity PID regulation
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motors will brake when zero power is applied (rather than coast)
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lastM1Encoder = robot.bulkData2.getMotorCurrentPosition(motor1);
        //lastM2Encoder = robot.bulkData2.getMotorCurrentPosition(motor2);
        lastM1Encoder = 0;
        lastM2Encoder = 0;

        if (debuggingMode) {
            dataLogger = new DataLogger(moduleSide + "ModuleLog");
            dataLogger.addField("Trans Vector FC X");
            dataLogger.addField("Trans Vector FC Y");
            dataLogger.addField("Rot Vector X");
            dataLogger.addField("Rot Vector Y");
            dataLogger.addField("Target Vector X");
            dataLogger.addField("Target Vector Y");
            dataLogger.addField("Module Orientation");
            dataLogger.addField("Reversed");
            dataLogger.addField("Power Vector X (TRANS)");
            dataLogger.addField("Power Vector Y (ROT)");
            dataLogger.addField("Motor 1 Power");
            dataLogger.addField("Motor 2 Power");
            dataLogger.addField("Motor 1 Encoder");
            dataLogger.addField("Motor 2 Encoder");
            dataLogger.newLine();
        }
    }

    //defaults to false for debugging mode (optional parameter)
    public DriveModule(Robot robot, ModuleSide moduleSide) {
        this(robot, moduleSide, false);
    }

    //this method updates the target vector for the module based on input from auto/teleop program
    public void updateTarget (Vector2d transVec, double rotMag) { //translation vector and rotation magnitude
        //transVec = transVec.rotate(-90); // to change to heading instead of cartesian from joystick

        //converts the translation vector from a robot centric to a field centric one
        Vector2d transVecFC = transVec.rotateBy(robot.getRobotHeading().getAngle(Angle.AngleType.ZERO_TO_360_HEADING), Angle.Direction.COUNTER_CLOCKWISE); //was converted robot heading, was clockwise

        //vector needed to rotate robot at the desired magnitude
        //based on positionVector of module (see definition for more info)
        Vector2d rotVec = positionVector.normalize(rotMag).rotateBy(90, Angle.Direction.COUNTER_CLOCKWISE); //theoretically this should be rotated 90, not sure sure it doesn't need to be

        //combine desired robot translation and robot rotation to get goal vector for the module
        Vector2d targetVector = transVecFC.add(rotVec);

        //allows modules to reverse power instead of rotating 180 degrees
        //example: useful when going from driving forwards to driving backwards
//        int directionMultiplier = -1; //was positive 1
        if (reversed) { //reverse direction of translation because module is reversed
            targetVector = targetVector.reflect();
//            directionMultiplier = 1;
        }

        if (debuggingMode) {
            dataLogger.addField(transVecFC.getX());
            dataLogger.addField(transVecFC.getY());
            dataLogger.addField(rotVec.getX());
            dataLogger.addField(rotVec.getY());
            dataLogger.addField(targetVector.getX());
            dataLogger.addField(targetVector.getY());
            dataLogger.addField(getCurrentOrientation().getAngle());
            dataLogger.addField(reversed);
        }

        //calls method that will apply motor powers necessary to reach target vector in the best way possible, based on current position
        //goToTarget(targetVector, directionMultiplier);
        goToTarget(targetVector);

        if (debuggingMode) {
            robot.telemetry.addData(moduleSide + " REVERSED: ", reversed);
            robot.telemetry.addData(moduleSide + " Trans Vec FC: ", transVecFC);
            robot.telemetry.addData(moduleSide + " Rot Vec: ", rotVec);
        }
    }

    //used in place of updateTarget() when the "absolute heading mode" is being used in TeleOp
    //"absolute heading mode" means the angle of the rotation joystick is used to determine the desired angle of the robot
    public void updateTargetAbsRotation (Vector2d transVec, Angle targetHeading, double scaleFactor) { //translation vector and rotation magnitude
        Angle robotHeading = robot.getRobotHeading(); //todo: this changed
        double rotMag = getRotMag(targetHeading, robotHeading) * scaleFactor;
        updateTarget(transVec, rotMag);
    }

    //used ONLY for abs heading mode (alternative teleop control style)
    public double getRotMag (Angle targetHeading, Angle robotHeading)
    {
        robot.telemetry.addData("Difference between joystick and robot", targetHeading.getDifference(robotHeading));
        robot.telemetry.addData("Direction from robot to joystick", robotHeading.directionTo(targetHeading));

        double unsignedDifference = RobotUtil.scaleVal(targetHeading.getDifference(robotHeading),15, 60, .3, 1);
        //todo: check if below line is causing problem
        if (robotHeading.directionTo(targetHeading) == Angle.Direction.CLOCKWISE) {
            return unsignedDifference * -1;
        } else {
            return unsignedDifference;
        }
    }

    //sets motor powers for robot to best approach given target vector
    public void goToTarget (Vector2d targetVector) {
        double powerScale = targetVector.getMagnitude();
        double angleDifference = targetVector.getAngle().getDifference(getCurrentOrientation());
        if (angleDifference > 110) { //todo: change this constant
            targetVector = targetVector.reflect();
            angleDifference = targetVector.getAngle().getDifference(getCurrentOrientation());
        }
        Angle.Direction directionToRotate = getCurrentOrientation().directionTo(targetVector.getAngle());
        if (directionToRotate == Angle.Direction.COUNTER_CLOCKWISE) angleDifference *= -1;

        double slope = 2 * powerScale / ANGLE_OF_MAX_MODULE_ROTATION_POWER;
        double motor1power, motor2power;
        if (angleDifference > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
            motor1power = motor2power = powerScale;
        } else if (angleDifference > 0) {
            motor1power = powerScale;
            motor2power = slope * angleDifference - powerScale;
        } else if (angleDifference > -ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
            motor1power = -powerScale;
            motor2power = slope * angleDifference + powerScale;
        } else {
            motor1power = motor2power = -powerScale;
        }

        motor1.setPower(motor1power);
        motor2.setPower(motor2power);


        if (debuggingMode) {
            robot.telemetry.addData(moduleSide + " Target Vector Angle: ", targetVector.getAngleDouble(Angle.AngleType.ZERO_TO_360_HEADING));
            robot.telemetry.addData(moduleSide + " Current orientation: ", getCurrentOrientation().getAngle());
            dataLogger.addField(robot.bulkData2.getMotorCurrentPosition(motor1));
            dataLogger.addField(robot.bulkData2.getMotorCurrentPosition(motor2));
            dataLogger.newLine();
        }
    }


//    //for pure module rotation (usually used for precise driving in auto)
//    public void rotateModule (Vector2d direction, boolean fieldCentric) {
//        //converts robot heading to the angle type used by Vector2d class
//        Angle convertedRobotHeading = robot.getRobotHeading().convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN);
//
//        //pass 0 as moveComponent
//        //todo: check if fixes broke this
//        Vector2d directionFC = direction.rotateTo(robot.getRobotHeading()); //was converted robot heading
//
//        //ADDED
//        if (reversed) { //reverse direction of translation because module is reversed
//            directionFC = directionFC.reflect();
//            direction = direction.reflect();
//        }
//
//        Vector2d powerVector;
//        if (fieldCentric) {
//            powerVector = new Vector2d(0, getPivotComponent(directionFC, getCurrentOrientation())); //order important here
//        } else {
//            powerVector = new Vector2d(0, getPivotComponent(direction, getCurrentOrientation())); //order important here
//        }
//        setMotorPowers(powerVector);
//        if (debuggingMode) {
//            robot.telemetry.addData(moduleSide + " Power Vector: ", powerVector);
//            robot.telemetry.addData(moduleSide + " Current orientation: ", getCurrentOrientation().getAngle());
//        }
//    }

    //does not need to be called at the start of every program
    //separate OpMode called ResetEncoders calls this method
    public void resetEncoders () {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //returns module orientation relative to ROBOT (not field) in degrees and NEG_180_TO_180_HEADING type
    public Angle getCurrentOrientation() {
        robot.telemetry.addData(moduleSide + "Motor 1 Encoder", robot.bulkData2.getMotorCurrentPosition(motor1));
        robot.telemetry.addData(moduleSide + "Motor 2 Encoder", robot.bulkData2.getMotorCurrentPosition(motor2));
        double rawAngle = (double)(robot.bulkData2.getMotorCurrentPosition(motor2) + robot.bulkData2.getMotorCurrentPosition(motor1))/2.0 * DEGREES_PER_TICK; //motor2-motor1 makes ccw positive (?)
        return new Angle(rawAngle, Angle.AngleType.ZERO_TO_360_HEADING);
    }


    //TRACKING METHODS
    //used for robot position tracking

    //new position tracking
    public Vector2d updatePositionTracking (Telemetry telemetry) {
        double newM1Encoder = robot.bulkData2.getMotorCurrentPosition(motor1);
        double newM2Encoder = robot.bulkData2.getMotorCurrentPosition(motor2);

        //angles are in radians
        Angle startingAngleObj = new Angle((lastM1Encoder + lastM2Encoder)/2.0 * DEGREES_PER_TICK, Angle.AngleType.ZERO_TO_360_HEADING);
        Angle finalAngleObj = new Angle((newM1Encoder + newM2Encoder)/2.0 * DEGREES_PER_TICK, Angle.AngleType.ZERO_TO_360_HEADING);
        double averageAngle = Math.toRadians(Angle.getAverageAngle(startingAngleObj, finalAngleObj).getAngle(Angle.AngleType.ZERO_TO_360_CARTESIAN)); //was 180 heading

        double startingPosition = (lastM1Encoder - lastM2Encoder)/2.0  * CM_PER_TICK;
        double finalPosition = (newM1Encoder - newM2Encoder)/2.0 * CM_PER_TICK;
        positionChange = finalPosition - startingPosition;
        //if (reversed) positionChange *= -1; //todo: test this change (including how it may affect heading tracking)

        Vector2d displacementVec;
        double deltaYPos = Math.sin(averageAngle) * positionChange; //was x
        double deltaXPos = Math.cos(averageAngle) * positionChange; //was y
        displacementVec = new Vector2d(-deltaXPos, -deltaYPos); //added negatives based on testing results

        if (debuggingMode) {
            telemetry.addData("Position change: ", positionChange);
            telemetry.addData("Average angle: ", averageAngle);
            telemetry.addData(moduleSide + " Displacement vector: ", displacementVec);
            telemetry.addData(moduleSide + " Delta X Pos: ", displacementVec.getX());
            telemetry.addData(moduleSide + " Delta Y Pos: ", displacementVec.getY()); //was printing the final position instead...
        }

        lastM1Encoder = newM1Encoder;
        lastM2Encoder = newM2Encoder;

        return displacementVec;
    }
}
