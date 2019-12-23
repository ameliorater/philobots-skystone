package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.ROTATE_MODULES;

enum ModuleSide {LEFT, RIGHT}

public class DriveController {
    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;

    double robotAbsXPos;
    double robotAbsYPos;
    double robotAbsHeading; //0 to 360 heading-style (clockwise is positive, 0 is straight ahead)

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    final double ROBOT_WIDTH = 18*2.54;

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //tolerance for robot rotation (in degrees)
    public final double ALLOWED_ROBOT_ROT_ERROR = 5; //was 3

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //minimum drive power (ever)
    //TODO: actually set this to minimum possible power
    double MIN_DRIVE_POWER = 0.3;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR = 0.2;

    //default timeouts
    public double DEFAULT_TIMEOUT_ROT_MODULES = 750; //was 500
    public double ROTATE_ROBOT_TIMEOUT = 3000;
    public double DRIVE_TIMEOUT = 4000;

    public DriveController(Robot robot) {
        this.robot = robot;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT);

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();

        //todo: change to parameter
        robotAbsXPos = 0;
        robotAbsYPos = 0;
        robotAbsHeading = 0;
    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2) {
//        if (joystick1.getMagnitude() == 0) update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
//        else update(joystick1, -joystick2.getX() * ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR);
        if (joystick1.getMagnitude() == 0) updateAbsRotation(joystick1, joystick2);
        else updateAbsRotation(joystick1, joystick2);
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);

    }
    public void updateAbsRotation(Vector2d translationVector, Vector2d joystick2) {
        Angle targetAngle = joystick2.getAngleAngle();
        if (joystick2.getMagnitude() > 0.1 && joystick2.getAngleAngle().getDifference(robot.getRobotHeading()) > 5) {
            moduleLeft.updateTargetAbsRotation(translationVector, targetAngle);
            moduleRight.updateTargetAbsRotation(translationVector, targetAngle);
        } else {
            moduleLeft.updateTarget(translationVector, 0);
            moduleRight.updateTarget(translationVector, 0);

        }
    }

    //AUTONOMOUS METHODS
    //do NOT call in a loop

    //speed should be scalar from 0 to 1
    public void drive(Vector2d direction, double cmDistance, double speed, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance/2.0; //BAD :(
        double startTime = System.currentTimeMillis();
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        //updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && /*System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && */linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();

            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //aligns modules before driving but DOES NOT fix them (modules will adjust orientation while driving)
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        drive(direction, cmDistance, speed, false, true, linearOpMode);
    }


//    public void driveWhileAdjusting(double distance) {
//        while (getDistanceTraveled() < distance) {
//            double distanceDifference = moduleRight.getDistanceTraveled() - moduleLeft.getDistanceTraveled();
//            double powerDifference = RobotUtil.mapToLineThroughOrigin(distanceDifference, 0.4, 30);
//            //TODO change the values above to constants and tune
//            double averagePower = RobotUtil.scaleVal(distance - getDistanceTraveled(), minDistance, maxDistance, minPower, maxPower);
//            moduleLeft.driveWhileAdjusting(averagePower - powerDifference / 2);
//            moduleRight.driveWhileAdjusting(averagePower + powerDifference / 2);
//            //TODO account for the fact that power could be over 1 and scale
//        }
//    }

    //speed should be scalar from 0 to 1
    public void driveWithRange (Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        if (frontSensor) stopAtDistance = stopAtDistance + 10; //account for inset into robot
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        boolean continueLoop;

        double startTime = System.currentTimeMillis();
        resetDistanceTraveled();

        do {
            //loop stop condition
            if (forward) continueLoop = robot.getRange(frontSensor) > stopAtDistance;
            else continueLoop = robot.getRange(frontSensor) < stopAtDistance;

            //slows down drive power in certain range
            if (robot.getRange(frontSensor) - stopAtDistance < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(robot.getRange(frontSensor) - stopAtDistance, 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking();
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        } while (continueLoop && System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive());

        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //defaults to fix modules and align modules both TRUE
    public void driveWithRange(Vector2d direction, double stopAtDistance, boolean forward, boolean frontSensor, double speed, double timeout, LinearOpMode linearOpMode) {
        driveWithRange(direction, stopAtDistance, forward, frontSensor, speed, timeout,true, true, linearOpMode);
    }

    //speed should be scalar from 0 to 1
    public void driveWithTimeout(Vector2d direction, double cmDistance, double speed, double timeout, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance/2.0; //BAD :(
        double startTime = System.currentTimeMillis();
        double initalSpeed = speed;

        //turns modules to correct positions for straight driving
        if (alignModules) rotateModules(direction, true, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else setRotateModuleMode(ROTATE_MODULES); //reset mode

        resetDistanceTraveled();
        //updateTracking(); //ADDED

        while (getDistanceTraveled() < cmDistance && System.currentTimeMillis() - startTime < timeout && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    //aligns modules before driving but DOES NOT fix them (modules will adjust orientation while driving)
    public void driveWithTimeout(Vector2d direction, double cmDistance, double speed, double timeout, LinearOpMode linearOpMode) {
        driveWithTimeout(direction, cmDistance, speed, timeout, false, true, linearOpMode);
    }



    public void rotateRobot(Angle targetAngle, double power, LinearOpMode linearOpMode) {
        double startTime = System.currentTimeMillis();
        rotateModules(Vector2d.FORWARD, false, DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //rotateModules
        int iterations = 0;
        boolean isNegativeRotation = robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE;

        double absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
        while (absHeadingDiff > ALLOWED_MODULE_ROT_ERROR && linearOpMode.opModeIsActive() && iterations < MAX_ITERATIONS_ROBOT_ROTATE /*&& System.currentTimeMillis() - startTime < ROTATE_ROBOT_TIMEOUT*/) {
            absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
            double rotMag = RobotUtil.scaleVal(absHeadingDiff, 0, 25, 0, power); //was max power 1 - WAS 0.4 max power

            if (robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE) {
                update(Vector2d.ZERO, -rotMag);
                if (!isNegativeRotation) iterations++;
            } else {
                update(Vector2d.ZERO, rotMag);
                if (isNegativeRotation) iterations++;
            }
            linearOpMode.telemetry.addData("Rotating ROBOT", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        }
        update(Vector2d.ZERO, 0);
    }

    public void rotateRobot(Angle targetAngle, LinearOpMode linearOpMode) {
        rotateRobot(targetAngle, 0.4, linearOpMode);
    }


    //both modules must be within allowed error for method to return
    public void rotateModules(Vector2d direction, boolean fieldCentric, double timemoutMS, LinearOpMode linearOpMode) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        double startTime = System.currentTimeMillis();
        do {
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getRealAngle()); //was getRealAngle() (don't ask)
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getRealAngle());
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleLeft.rotateModule(direction, fieldCentric);
            moduleRight.rotateModule(direction, fieldCentric);

            linearOpMode.telemetry.addData("Rotating MODULES", "");
            linearOpMode.telemetry.addData("Top level module left difference", moduleLeftDifference);
            linearOpMode.telemetry.addData("Top level module right difference", moduleRightDifference);
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
        } while ((moduleLeftDifference > ALLOWED_MODULE_ROT_ERROR || moduleRightDifference > ALLOWED_MODULE_ROT_ERROR) && linearOpMode.opModeIsActive() && System.currentTimeMillis() < startTime + timemoutMS);
        update(Vector2d.ZERO, 0);
    }



    //TRACKING METHODS
    //methods for path length tracking in autonomous (only useful for driving in straight lines)

    //new tracking method
    public void updatePositionTracking (Telemetry telemetry) {
        Vector2d rightDisp = moduleRight.updatePositionTracking(telemetry);
        Vector2d leftDisp = moduleLeft.updatePositionTracking(telemetry);

        rightDisp.setX(rightDisp.getX() + ROBOT_WIDTH/2);
        leftDisp.setX(leftDisp.getX() - ROBOT_WIDTH/2);

        Vector2d robotCenterDisp = new Vector2d((rightDisp.getX() + leftDisp.getX())/2, (rightDisp.getY() + leftDisp.getY())/2);
        robotCenterDisp.rotate(robotAbsHeading); //make field centric using previous heading
        robotAbsXPos += robotCenterDisp.getX();
        robotAbsYPos += robotCenterDisp.getY();

        Vector2d wheelToWheel = new Vector2d(rightDisp.getX() - leftDisp.getX(), rightDisp.getY() - leftDisp.getY()); //left to right
        //todo: check that get angle methods return correct things
        double robotAngleChange = wheelToWheel.getAngle();
        robotAbsHeading -= robotAngleChange; //minus because clockwise vs. counterclockwise (which one is positive changes)
        robotAbsHeading = robotAbsHeading % 360; //todo: check if we want this or the Python mod function

        telemetry.addData("Robot X Position: ", robotAbsXPos);
        telemetry.addData("Robot Y Position: ", robotAbsYPos);
    }

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }


    public void setRotateModuleMode(DriveModule.RotateModuleMode rotateModuleMode) {
        moduleLeft.rotateModuleMode = rotateModuleMode;
        moduleRight.rotateModuleMode = rotateModuleMode;
    }


    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }
}