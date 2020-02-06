package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

import static org.firstinspires.ftc.teamcode.Constants.IntakeState.INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeState.OUTTAKE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeState.STOP;

@Autonomous(name = "Tracking Test", group = "Linear Opmode")
public class TrackingAutoTest extends LinearOpMode {

    public TrackingAutoTest(boolean isBlue, boolean twoSkystone) {
        this.isBlue = isBlue;
        this.twoSkystone = twoSkystone;
    }

    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    SkystoneCV cv;
    boolean isBlue;
    boolean twoSkystone;
    double DEFAULT_POWER = 1; //was 0.6 for most
    double SLOW_POWER = 0.35; //was 0.35

    double TILE = 24 * 2.54;
    double ROBOT = 45;

    double RED_CROSS_FIELD_Y = -95;

    public void runOpMode() {

        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // set starting position and orientation of the robot
        simpleTracking.setPosition(-90, isBlue ? 157.5 : -157.5);
        simpleTracking.setOrientationDegrees(isBlue ? 180 : 0);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        cv = new SkystoneCV("Webcam 1", new Point(30, 130), new Point(110, 130), new Point(190, 130), this);
        cv.init(SkystoneCV.CameraType.WEBCAM);

        simpleTracking.setModuleOrientation(robot);


        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        long startTime = System.currentTimeMillis();

        //START AUTO

        // set SCARA to be inside robot
        double lastTime = getRuntime();
        double currentTime = getRuntime();
        while (opModeIsActive() && !robot.currentClawPosition.moveSequence(robot.controller.DELIVERY_TO_INSIDE_ROBOT, currentTime - lastTime)) {
            robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
            robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);

            lastTime = currentTime;
            robot.wait(10, this);
            currentTime = getRuntime();
        }
        // make sure the last position gets sent
        robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
        robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);


        //identify skystone
        SkystoneCV.StonePosition skystonePosition = cv.getSkystonePosition();
        double stonePosition;
        if (isBlue) {
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                stonePosition = -67;
            } else if (skystonePosition == SkystoneCV.StonePosition.LEFT) {
                stonePosition = -56;
            } else {
                stonePosition = -97;
            }
        } else {
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                stonePosition = -77;
            } else if (skystonePosition == SkystoneCV.StonePosition.RIGHT) {
                stonePosition = -56;
            } else {
                stonePosition = -97;
            }
        }

        //close openCV
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();

        // go to first stone
        moveTo(stonePosition, (isBlue ? 75 : -75), isBlue ? 225 : 315, 0.5, 5, 3000);
        simplePathFollow.stop(robot);
        robot.hungryHippoRetract(); //pull block in
        robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
        //go forward to grab block
        moveTo(stonePosition, isBlue ? 40 : -40, isBlue ? 225 : 315, SLOW_POWER, 5, 3000);
        simplePathFollow.stop(robot);
        robot.hungryHippoExtend(); //pull hungry hippo back in

        //intake stone
        robot.wait(500, this); //was 1000
        robot.moveIntake(STOP);

        moveTo(stonePosition, isBlue ? 90 : RED_CROSS_FIELD_Y, isBlue ? 180 : 0, DEFAULT_POWER, 2); //WAS 95
        moveTo(stonePosition, isBlue ? 90 : RED_CROSS_FIELD_Y, 270, DEFAULT_POWER, 1); //WAS ALSO 95
        simplePathFollow.stop(robot);

        // move to platform
        moveWithIMU(120, isBlue ? 90 : RED_CROSS_FIELD_Y, 270, 0.6, 5, 6000);

        moveTo(120, isBlue ? 90 : RED_CROSS_FIELD_Y, isBlue ? 0 : 180, DEFAULT_POWER, 5);
        simplePathFollow.stop(robot);
        moveWithRangeSensorTo(120, isBlue ? 35 : -35, isBlue ? 0 : 180, 0.4, 5, 3000);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);

        robot.moveIntake(OUTTAKE);
        robot.wait(500, this); //was 1500 timeout
        robot.moveIntake(STOP);

        moveTo(120, isBlue ? 125 : -125, isBlue ? 0 : 180, DEFAULT_POWER, 5, 5000);
        simplePathFollow.stop(robot);
        moveTo(50, isBlue ? 125 : -125, isBlue ? 225 : 315, DEFAULT_POWER, 2, 2500); //pivot platform  //WAS 80 //WAS 4000 timeout
        moveTo(110, isBlue ? 140 : -140, 270, DEFAULT_POWER, 5, 2000);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);


        //TWO SKYSTONE
        if (twoSkystone /*&& System.currentTimeMillis() - startTime < 25*/) { //at least 5 seconds left
            moveTo(50, 95 * (isBlue ? 1 : -1), 270, 0.6, 5); //position for cross-field drive
            moveWithIMU(-100, 95 * (isBlue ? 1 : -1), 270, 0.6, 5); //align next to stone
            moveTo(-93, 52 * (isBlue ? 1 : -1), 270, 0.6, 5); //was x = -100
            robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
            moveTo(-120, 52 * (isBlue ? 1 : -1), 270, 0.3, 5);
            robot.moveIntake(STOP);
            moveTo(-121, 95 * (isBlue ? 1 : -1), 270, 0.6, 5);
            moveTo(-121, 95 * (isBlue ? 1 : -1), 90, 0.3, 5); //rotate
            moveTo(70, 95 * (isBlue ? 1 : -1), 90, 0.6, 5);
            robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW);
            moveTo(0, 95 * (isBlue ? 1 : -1), 90, 0.6, 5);
            robot.moveIntake(STOP);

//            double additionalDist = 0;
//            if (skystonePosition == SkystoneCV.StonePosition.CENTER) additionalDist = 8 * 2.54;
//            if (skystonePosition == SkystoneCV.StonePosition.LEFT) additionalDist = 16 * 2.54;
//            double position = -(1.75 * TILE + additionalDist); //was 2 * TILE
//
//            simplePathFollow.stop(robot);
////            waitForButton();
//            simplePathFollow.stop(robot);
////            waitForButton();
//
//            moveWithIMU(-100, isBlue ? 90 : -98, 270, 0.6, 5); //align next to stone
//            robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
//            moveTo(position, (TILE - 4 * 2.54) * (isBlue ? 1 : -1), 270, 0.6, 5); //get in front of stone
//            moveTo(position - 8 * 2.54, (TILE - 4 * 2.54) * (isBlue ? 1 : -1), 270, 0.6, 5); //intake stone
//            robot.moveIntake(STOP);
//
//            //backtrack and score
//            moveTo(position - 8 * 2.54, isBlue ? 90 : RED_CROSS_FIELD_Y, 270, 0.6, 5); //prepare to drive back
//            moveWithIMU(1.5 * TILE, isBlue ? 90 : RED_CROSS_FIELD_Y, 90, 0.6, 5); //cross-field drive
//            robot.moveIntake(OUTTAKE);
//            moveWithIMU(0, isBlue ? 90 : RED_CROSS_FIELD_Y, 90, 0.6, 5); //cross-field drive
//            robot.moveIntake(STOP);
//
//            moveTo(-1.5 * TILE, 1.5 * TILE + 4, 270, 0.6, 5); //get in front of foundation
//
            //score block

        } else {
            //park
            moveWithIMU(0, isBlue ? 95 : -95, 270, 0.6, 5);
        }

        simplePathFollow.stop(robot);
    }

    private void moveTo(double x, double y, double orientation, double speed, double threshold) {
        boolean done = false;
        while (!done && opModeIsActive()) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithIMU(double x, double y, double orientation, double speed, double threshold) {
        boolean done = false;
        while (!done && opModeIsActive()) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot, true, isBlue);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithIMU(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot, true, isBlue);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    // with timeout
    private void moveTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithRangeSensorTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold) ||
                    (robot.backRangeSensor.getDistance(DistanceUnit.CM) < 2);
            logTelemetry();
        }
    }

    //move relative to the current position
    private void move(double x, double y, double orientation, double speed, double threshold) {
        moveTo(simpleTracking.lastRobotX + x, simpleTracking.lastRobotY + y, simpleTracking.lastRobotOrientation + orientation, speed, threshold);
    }

    public void updateOrientationWithIMU(boolean isBlue) {
        double angle = robot.getRobotHeading().getAngle();
        if (isBlue) angle += 180;
        if (angle < 0) angle += 360;
        if (angle >= 360) angle -= 360;
        simpleTracking.setOrientationDegrees(angle);
    }

    public void waitForButton() {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("IMU Heading: ", robot.getRobotHeading());
            logTelemetry();
        }
    }

    private void logTelemetry() {
        telemetry.addData("Left Top encoder", simpleTracking.leftTopMotorPosition);
        telemetry.addData("Left Bottom encoder", simpleTracking.leftBottomMotorPosition);
        telemetry.addData("Left Distance", simpleTracking.lastleftDistance);
        telemetry.addData("Left Orientation", simpleTracking.lastLeftOrientation);

        telemetry.addData("Right Top encoder", simpleTracking.rightTopMotorPosition);
        telemetry.addData("Right Bottom encoder", simpleTracking.rightBottomMotorPosition);
        telemetry.addData("Right Distance", simpleTracking.lastRightDistance);
        telemetry.addData("Right Orientation", simpleTracking.lastRightOrientation);

        telemetry.addData("Target translation", simplePathFollow.targetTranslation.x + ", " + simplePathFollow.targetTranslation.y);
        telemetry.addData("Target rotation", simplePathFollow.targetRotation.x + ", " + simplePathFollow.targetRotation.y);
        telemetry.addData("Target Left Power", simplePathFollow.targetLeftPower.x + ", " + simplePathFollow.targetLeftPower.y);
        telemetry.addData("Target Right Power", simplePathFollow.targetRightPower.x + ", " + simplePathFollow.targetRightPower.y);

        telemetry.addData("Left Top Power", simplePathFollow.leftTopPower);
        telemetry.addData("Left Bottom Power", simplePathFollow.leftBottomPower);
        telemetry.addData("Right Top Power", simplePathFollow.rightTopPower);
        telemetry.addData("Right Bottom Power", simplePathFollow.rightBottomPower);

        telemetry.addData("Robot position", simpleTracking.lastRobotX + ", " + simpleTracking.lastRobotY);
        telemetry.addData("Robot orientation", simpleTracking.lastRobotOrientation);

        telemetry.update();
    }
}
