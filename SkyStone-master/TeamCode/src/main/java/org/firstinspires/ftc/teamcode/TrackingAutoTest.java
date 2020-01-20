package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

@Autonomous(name="Tracking Test", group="Linear Opmode")
public class TrackingAutoTest extends LinearOpMode {

    public TrackingAutoTest(boolean blue) {
        isBlue = blue;
    }

    public TrackingAutoTest() {
        isBlue = true;
    }

    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    FieldTracker fieldTracker;
    SkystoneCV cv;
    boolean isBlue;

    public void runOpMode() {

        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // optionally set starting position and orientation of the robot
        simpleTracking.setOrientationDegrees(0);
        simpleTracking.setPosition(0,0);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        cv = new SkystoneCV("Webcam 1", new Point(30, 130), new Point(110, 130), new Point(190, 130), this);
        cv.init(SkystoneCV.CameraType.WEBCAM);

        simpleTracking.setModuleOrientation(robot);

        // position intake wheels inside the robot
        robot.intakeServo1.setPosition(1.0);
        robot.intakeServo2.setPosition(0.0);


        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        // blue starting position
        simpleTracking.setPosition(-90, isBlue ? 157.5 : -157.5);
        simpleTracking.setOrientationDegrees(isBlue ? 180 : 0);
        robot.intakeServo1.setPosition(0.0);
        robot.intakeServo2.setPosition(1.0);
        robot.openGrabber();

        /*
        double stonePosition =
                position == SkystoneCV.StonePosition.CENTER ? -93 :
                        (position == SkystoneCV.StonePosition.LEFT ? -73 : -113);// depends on stone*/

        SkystoneCV.StonePosition position = cv.getSkystonePosition();
        double stonePosition;
        if (isBlue) {
            if (position == SkystoneCV.StonePosition.CENTER) {
                stonePosition = -90;
            } else if (position == SkystoneCV.StonePosition.LEFT) {
                stonePosition = -70;
            } else {
                stonePosition = -110;
            }
        } else {
            if (position == SkystoneCV.StonePosition.CENTER) {
                stonePosition = -81;
            } else if (position == SkystoneCV.StonePosition.RIGHT) {
                stonePosition = -61;
            } else {
                stonePosition = -101;
            }
        }

        //close openCV and start fieldTracker
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();
        //fieldTracker = new FieldTracker(hardwareMap, telemetry, true, false);

        // go to the center stone
        moveTo(stonePosition, (isBlue ? 85 : -85), isBlue ? 180 : 0, 0.5, 5);
        simplePathFollow.stop(robot);
        robot.armServo1.setPosition(.4);
        robot.armServo2.setPosition(.6);
        robot.hungryHippoRetract();
        robot.moveIntake(Constants.IntakeState.INTAKE,Constants.IntakeSpeed.SLOW);
        robot.wait(1000, this, simpleTracking);
        moveTo(stonePosition, isBlue ? 50 : -50, isBlue ? 180 : 0, 0.35, 5);
        simplePathFollow.stop(robot);
        robot.armServo1.setPosition(.7);
        robot.armServo2.setPosition(.3);
        robot.wait(1200,this, simpleTracking);
        robot.hungryHippoExtend();
        robot.moveIntake(Constants.IntakeState.STOP);
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);

        robot.wait(1000, this, simpleTracking);
        moveTo(stonePosition, isBlue ? 115 : -115, isBlue ? 180: 0, 0.6, 2); //WAS 95
        moveTo(stonePosition, isBlue ? 95: -95, 270, 0.6, 1);
        simplePathFollow.stop(robot);
        //robot.wait(250, this); //removed 1-20

        //waitForButton();

        // move to platform
        moveWithIMU(120, isBlue ? 95 : -95, 270, 0.6, 5);

        moveTo(120, isBlue ? 90: -90, isBlue ? 0:180, 0.6, 5);
        simplePathFollow.stop(robot);
        moveWithRangeSensorTo(120, isBlue ? 35: -35, isBlue ? 0 : 180, 0.4, 5, 3000);
//        moveTo(isBlue ? 120 : -213, 60, 0, 0.4, 5);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);
        robot.wait(200, this, simpleTracking);
        robot.closeGrabber();
        robot.wait(200, this, simpleTracking);
        robot.openGrabber();
        robot.wait(200, this, simpleTracking);
        robot.closeGrabber();
        robot.wait(500, this, simpleTracking);
        robot.armServo1.setPosition(.3);
        robot.armServo2.setPosition(.7);
        robot.wait(1200, this, simpleTracking);
        robot.openGrabber();
        robot.armServo1.setPosition(.7);
        robot.armServo2.setPosition(.3);

        robot.wait(1000, this, simpleTracking);
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);
        moveTo(120, isBlue ? 125:-125, isBlue ?0:180, 0.6, 5, 5000);
        simplePathFollow.stop(robot);
        moveTo(50, isBlue ?125 : -125, isBlue ? 245 : 295, 1.0, 2); //pivot platform  //WAS 80
        moveTo(110, isBlue ? 125 : -125, 270, 0.6, 5, 2000);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);

        //waitForButton();

        moveTo(60, isBlue  ? 80 : -80, 270, 0.6, 5);
        moveTo(0, isBlue ? 80 : -80, 270, 0.6, 5);
        simplePathFollow.stop(robot);

        // test translate
//        moveTo(0, 180, 0, 0.6, 10);
//        moveTo(-240, 180, 0, 0.6, 10);
//        moveTo(-240, 0, 0, 0.6, 10);
//        moveTo(0, 0, 0, 0.6, 10);
        // test rotate
//        moveTo(0, 0, 180, 0.5, 5);

//        while (opModeIsActive()) {
//            logTelemetry();
//        }

        // test rotate and translate
//        moveTo(0, 100, 90, 0.4, 5);
        // stop all motors
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
                    ( robot.backRangeSensor.getDistance(DistanceUnit.CM) < 2);
            logTelemetry();
        }
    }

    //move relative to the current position
    private void move(double x, double y, double orientation, double speed, double threshold) {
        moveTo(simpleTracking.lastRobotX + x, simpleTracking.lastRobotY + y, simpleTracking.lastRobotOrientation + orientation, speed, threshold);
    }

    public void updateOrientationWithIMU(boolean isBlue) {
        double angle =  robot.getRobotHeading().getAngle();
        if (isBlue) angle += 180;
        if (angle < 0) angle += 360;
        if (angle >= 360) angle -= 360;
        simpleTracking.setOrientationDegrees(angle);
    }

    public void waitForButton () {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("Vuforia location: \n", fieldTracker.getTargetInfo());
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
