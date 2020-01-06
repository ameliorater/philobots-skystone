package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.DriveModule.RotateModuleMode.ROTATE_MODULES;

enum ModuleSide {LEFT, RIGHT}

public class DriveController {
    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;

    Position robotPosition;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    final double WHEEL_TO_WHEEL_CM = 32.5; //in cm (was 18*2.54)

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
    public final double ROBOT_ROTATION_SCALE_FACTOR_ABS = 1;
    public final double ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS = 1;


    //default timeouts
    public double DEFAULT_TIMEOUT_ROT_MODULES = 750; //was 500
    public double ROTATE_ROBOT_TIMEOUT = 3000;
    public double DRIVE_TIMEOUT = 4000;

    //drive to position constants
    //todo: tune these constants
    double MAX_AUTO_DRIVE_FACTOR = 1;
    double MIN_AUTO_DRIVE_FACTOR = 0.1;
    double MAX_AUTO_ROTATE_FACTOR = 0.5;
    double MIN_AUTO_ROTATE_FACTOR = 0.1;

//    //Vuforia field tracking tools:
//    //This is the allowed distance to a target
//    public final double ALLOWED_DISTANCE_TO_TARGET = 5; //todo change this value
//    //This is the object to get the position on field
//    public FieldTracker vuforiaTracker = new FieldTracker(robot.hardwareMap, robot.telemetry, true, false);

    public DriveController(Robot robot, boolean debuggingMode) {
        this.robot = robot;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT, debuggingMode);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT, debuggingMode);

        //todo: change to parameter
        robotPosition = new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING));
    }

    //defaults to debugging mode off
    public DriveController(Robot robot) {
        this(robot, false);
    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2, boolean absHeadingMode) {
        if (absHeadingMode) {
            if (joystick1.getMagnitude() == 0)
                updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_SCALE_FACTOR_ABS);
            else updateAbsRotation(joystick1, joystick2, ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR_ABS);
        } else {
            if (joystick1.getMagnitude() == 0) update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
            else update(joystick1, -joystick2.getX() * ROBOT_ROTATION_WHILE_TRANS_SCALE_FACTOR);
        }
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    //this method is for "power-based rotation mode"
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
    }
    public void updateAbsRotation(Vector2d translationVector, Vector2d joystick2, double scaleFactor) {
        Angle targetAngle = joystick2.getAngle(); //was + .convertAngle(Angle.AngleType.NEG_180_TO_180_HEADING)
        if (joystick2.getMagnitude() > 0.1 && targetAngle.getDifference(robot.getRobotHeading()) > 3) {
            moduleLeft.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
            moduleRight.updateTargetAbsRotation(translationVector, targetAngle, scaleFactor);
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

    public void driveToPosition(Position currentPosition, Position targetPosition) {
        double totalXDistance = currentPosition.getXDifference(targetPosition);
        double totalYDistance = currentPosition.getYDifference(targetPosition);
        double totalHeadingDifference = currentPosition.getHeadingDifference(targetPosition);

        Vector2d translationDirection = currentPosition.getDirectionTo(targetPosition);
        Angle.Direction rotationDirection = currentPosition.getRotationDirectionTo(targetPosition);

        while (!targetPosition.withinRange(currentPosition, 5, 5, 5)){
            //scale speeds based on remaining distance from target, bounded by 0 and original distance
            //at the very beginning, x & y trans and rot will be max speed (different for each)
            //at the end, all three speeds will be min speed (different for each)
            //in between, the speeds will scale linearly depending on distance from target position

            double xPower = RobotUtil.scaleVal(currentPosition.getXDifference(targetPosition),
                    0, totalXDistance, MIN_AUTO_DRIVE_FACTOR, MAX_AUTO_DRIVE_FACTOR);

            double yPower = RobotUtil.scaleVal(currentPosition.getYDifference(targetPosition),
                    0, totalYDistance, MIN_AUTO_DRIVE_FACTOR, MAX_AUTO_DRIVE_FACTOR);

            double rotationPower = RobotUtil.scaleVal(currentPosition.getHeadingDifference(targetPosition),
                    0, totalHeadingDifference, MIN_AUTO_ROTATE_FACTOR, MAX_AUTO_ROTATE_FACTOR);

            //todo: may need to batch normalize all three somehow (x and y should be automatically normalized)
            Vector2d translationVector = new Vector2d(xPower * translationDirection.getX(), yPower * translationDirection.getY());
            if (rotationDirection == Angle.Direction.COUNTER_CLOCKWISE) {
                //todo: check CCW vs CW
                rotationPower *= -1;
            }

            update(translationVector, rotationPower);
        }
    }

    /*
    //Methods for moving with position tracking (Vuforia targets); UNTESTED
    public Position2D getCurrentPositionOnField() {
        TargetInfo info = vuforiaTracker.getTargetInfo();
        return info == null ? null : new Position2D(info.xPosition, info.yPosition);
    }

    public Vector2d getVectorToTarget(Position2D desiredPosition) {
        Position2D currentPosition = getCurrentPositionOnField();
        return currentPosition == null ? null : new Vector2d(desiredPosition.x - currentPosition.x, desiredPosition.y - currentPosition.y);
    }

    //The following two methods are used to go to a specific position; currently untested
    public void driveToPosition(double startx, double starty, double x, double y, double speed, long timeout, LinearOpMode linearOpMode) {
        driveToPosition(startx, starty, new Position2D(x, y), speed, timeout, linearOpMode);
    }

    public void driveToPosition(double startx, double starty, Position2D position, double speed, long timeout, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();

        Vector2d temp = getVectorToTarget(position);
        Vector2d vectorToTarget = temp == null ? new Vector2d(position.x - startx, position.y - starty) : temp;
        double distanceToTarget = vectorToTarget.getMagnitude();
        while (distanceToTarget < ALLOWED_DISTANCE_TO_TARGET && System.currentTimeMillis() - startTime < timeout) {
            if (distanceToTarget < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(distanceToTarget, 0, START_DRIVE_SLOWDOWN_AT_CM, MIN_DRIVE_POWER, speed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            updateTracking(); //WAS MOVED ABOVE
            update(vectorToTarget.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            updatePositionTracking(robot.telemetry); //update position tracking
>>>>>>> 8a70bf03325c5210c08d5c0c270ac79ea921df32

            temp = getVectorToTarget(position);
            if (temp != null) {
                vectorToTarget = temp;
                distanceToTarget = vectorToTarget.getMagnitude();
            }
        }
        update(Vector2d.ZERO, 0);
        setRotateModuleMode(ROTATE_MODULES); //reset mode
    }
    */

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
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngle()); //was getRealAngle() (don't ask)
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngle());
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

        //todo: incorporate Vuforia data & combination logic here

        //reset heading tracking with IMU if robot is not moving
        if (rightDisp.getMagnitude() < 0.01 && leftDisp.getMagnitude() < 0.01) {
            //use IMU for heading instead of encoders
            robotPosition.heading = robot.getRobotHeading();
        } else {
            //orientation tracking with encoders
            double arcLength = moduleRight.positionChange - moduleLeft.positionChange;
            double angleChange = arcLength * 360 / 2.0 / Math.PI / WHEEL_TO_WHEEL_CM;
            robotPosition.incrementHeading(angleChange);
        }

        rightDisp.setX(rightDisp.getX() + WHEEL_TO_WHEEL_CM /2);
        leftDisp.setX(leftDisp.getX() - WHEEL_TO_WHEEL_CM /2);

        Vector2d robotCenterDisp = new Vector2d((rightDisp.getX() + leftDisp.getX())/2, (rightDisp.getY() + leftDisp.getY())/2);
        robotCenterDisp = robotCenterDisp.rotateTo(robotPosition.heading); //make field centric using previous heading
        robotPosition.incrementX(robotCenterDisp.getX());
        robotPosition.incrementY(robotCenterDisp.getY());

        telemetry.addData("Robot X Position: ", robotPosition.x);
        telemetry.addData("Robot Y Position: ", robotPosition.y);
        telemetry.addData("Robot Abs Heading: ", robotPosition.heading);
    }

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //NOTE: this is the old method (straight line tracking only)
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