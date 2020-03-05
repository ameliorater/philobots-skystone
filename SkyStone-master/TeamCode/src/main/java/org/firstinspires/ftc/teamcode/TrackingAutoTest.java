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

    public TrackingAutoTest(boolean isBlue, boolean twoSkystone, boolean deliverWithScara) {
        this.isBlue = isBlue;
        this.twoSkystone = twoSkystone;
        this.deliverWithScara = deliverWithScara;
    }

    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    SkystoneCV cv;
    boolean isBlue, twoSkystone, deliverWithScara;
    double DEFAULT_POWER = 1.5; //was 0.6 for most
    double MID_POWER = 1.5; //was 0.75
    double SLOW_POWER = 0.45; //was 0.35

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

        if (deliverWithScara) {
            prepareToGrab();
        }

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        long startTime = System.currentTimeMillis();

        //START AUTO

        // set SCARA to be inside robot
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);

        //identify skystone
        SkystoneCV.StonePosition skystonePosition = cv.getSkystonePosition();
        telemetry.addData("Skystone at: ", skystonePosition);
        telemetry.update();

        double stonePosition = -70 + 5;
        if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
            stonePosition -= 8*2.54;
        } if ((skystonePosition == SkystoneCV.StonePosition.RIGHT && isBlue) || (skystonePosition == SkystoneCV.StonePosition.LEFT && !isBlue)) {
            stonePosition -= 16*2.54;
        }

        //close openCV
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();

        if (deliverWithScara) {
            //move lift up
            robot.moveLiftToPosition(175);
        }

        // go to first stone
        //moveTo(stonePosition, (isBlue ? 75 : -75), isBlue ? 225 : 315, 0.5, 5, 3000);
        moveTo(stonePosition, (isBlue ? 85 : -85), isBlue ? 180 : 0, 0.5, 5, 3000); //was 75 //was 85 //was 90
        simplePathFollow.stop(robot);

        robot.hungryHippoExtend();

        robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
        //go forward to grab block
        //moveTo(stonePosition, isBlue ? 25 : -25, isBlue ? 225 : 315, SLOW_POWER, 5, 3000); //was y = +- 40
        simplePathFollow.stop(robot);

        //intake stone
        robot.wait(1000, this); //was 1000
        robot.hungryHippoRetract();

        moveTo(stonePosition, (isBlue ? 75 : -75), isBlue ? 180 : 0, 0.5, 5, 3000); //was 75 //was 85

        double firstYDistance = 95; //was 95

        moveTo(stonePosition, firstYDistance * (isBlue ? 1 : -1), isBlue ? 180 : 0, 0.5, 2); //WAS 95 //was 90

        robot.wait(500, this); //added 3-2

        if (deliverWithScara) {
            robot.moveLiftToPosition(20); //lift down
        }

        moveTo(stonePosition, firstYDistance * (isBlue ? 1 : -1), 270, 0.4, 1); //WAS ALSO 95 //was 90 //was 0.5 speed
        simplePathFollow.stop(robot);

        // move to platform
        moveWithIMU(95, (firstYDistance + 15) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 6000); //was x = 120 //was 0.6 power //was 90 //was y = 110
        moveTo(120, firstYDistance * (isBlue ? 1 : -1), isBlue ? 0 : 180, DEFAULT_POWER, 5); //was 90
        simplePathFollow.stop(robot);
        moveWithRangeSensorTo(120, isBlue ? 35 : -35, isBlue ? 0 : 180, 0.4, 5, 3000);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);

        robot.moveIntake(STOP);

        if (deliverWithScara) {
            //deliver block to foundation
            deliverBlock(250);
        }

        //outtake just in case
        robot.moveIntake(OUTTAKE);
        robot.wait(750, this); //was 1500 timeout

        moveTo(120, isBlue ? 115 : -115, isBlue ? 0 : 180, DEFAULT_POWER, 5, 5000); //was y = 125
        simplePathFollow.stop(robot);
        moveTo(60, 95 * (isBlue ? 1 : -1), isBlue ? 245 : 295, DEFAULT_POWER, 2, 2500); //pivot platform  //WAS x = 50 //WAS angle = isBlue ? 225 : 315
        //moveTo(110, isBlue ? 140 : -140, 270, DEFAULT_POWER, 5, 2000); //push platform
        moveTo(90, isBlue ? 100 : -100, isBlue ? 245 : 295, DEFAULT_POWER, 5, 2000); //push platform
        robot.moveIntake(STOP);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);


        //TWO SKYSTONE
        if (deliverWithScara) prepareToGrab();
        if (twoSkystone /*&& System.currentTimeMillis() - startTime < 25*/) { //at least 5 seconds left
            double skystoneDistX = -100 - 5; //was -100 // was -100 + 10
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                skystoneDistX -= 8*2.54;
            } if ((skystonePosition == SkystoneCV.StonePosition.RIGHT && isBlue) || (skystonePosition == SkystoneCV.StonePosition.LEFT && !isBlue)) {
                skystoneDistX -= 16*2.54;
            }

            double secondYDistance = 85; //was 80
            double getBlockYDistance = 35; //was 30
            moveTo(50, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //position for cross-field drive (was x = 100) //was y = 90
            moveWithIMU(skystoneDistX, (secondYDistance + 10) * (isBlue ? 1 : -1), 270, MID_POWER, 5); //align next to stone (was x = 95)
            //if (deliverWithScara) robot.moveLiftToPosition(150); //added
            robot.moveLiftToPosition(0); //added
            moveTo(skystoneDistX, getBlockYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //was y = 57 //was y = 50 //was 0.6 power
            robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
            moveTo(skystoneDistX - 10, getBlockYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //was 0.3 power //was skyDist - 10
            robot.wait(1000, this); //wait for block to intake
            moveTo(skystoneDistX, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //was x-20
            robot.moveIntake(STOP); //moved one line down (give more time to intake)
            if (deliverWithScara) {
//                robot.moveLiftToPosition(20); //added
//                robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW); //just in case
//                moveWithIMU(85, (secondYDistance + 10) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000); //added timeout //changed to withIMU //was 90
//                robot.moveIntake(STOP);
//                moveTo(100, 130 * (isBlue ? 1 : -1), 270, 0.75, 5, 750); //was x = 120, y = 140
//                deliverBlock(250);
//                robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW); //just in case
//                moveTo(85, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000);
//                moveTo(0, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000);
//                robot.moveIntake(STOP);

                //outtake only
                //robot.moveLiftToPosition(20); //added
                //robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW); //just in case
                moveWithIMU(25, (secondYDistance + 10) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000); //added timeout //changed to withIMU //was 90
                moveWithIMU(25, (secondYDistance + 10) * (isBlue ? 1 : -1), isBlue ? 0 : 180, MID_POWER, 5, 3000); //added timeout //changed to withIMU //was 90
                robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW);
                //moveTo(100, 130 * (isBlue ? 1 : -1), 270, 0.75, 5, 750); //was x = 120, y = 140
                //deliverBlock(250);
                //robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW); //just in case
                //moveTo(85, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000);
                moveTo(0, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000);
                robot.moveIntake(STOP);

            } else {
                moveTo(skystoneDistX, secondYDistance * (isBlue ? 1 : -1), 90, 0.75, 5); //rotate
                moveWithIMU(30, secondYDistance * (isBlue ? 1 : -1), 90, MID_POWER, 5, 3000); //added timeout //changed to withIMU
                robot.moveIntake(OUTTAKE, Constants.IntakeSpeed.SLOW);
                moveTo(0, 90 * (isBlue ? 1 : -1), 90, 0.6, 5); //was y = 95
                robot.moveIntake(STOP);
            }
        } else {
            //park
            moveWithIMU(0, isBlue ? 75 : -75, 270, 0.6, 5);
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

    //move SCARA with sequence
    public void moveSCARA (SCARAController.Sequence sequence) {
        double lastTime = getRuntime();
        double currentTime = getRuntime();
        robot.setPlacerUp();
        while (opModeIsActive() && !robot.currentClawPosition.moveSequence(sequence, currentTime - lastTime)) {
            robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
            robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);

            lastTime = currentTime;
            robot.wait(10, this);
            currentTime = getRuntime();
        }
        // make sure the last position gets sent
        robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
        robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);
    }

    public void deliverBlock (int deliveryLiftPos) {
        robot.closeGrabber();
        robot.wait(750, this);
        robot.moveLiftToPosition(deliveryLiftPos); //was 175
        robot.wait(750, this);
        moveSCARA(robot.controller.INSIDE_ROBOT_TO_DELIVERY);
        robot.openGrabber();
        robot.wait(500, this);
        robot.grabberServo.setPosition(0.5);
        robot.setPlacerUp();
        robot.moveLiftToPosition(400); //was 175
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
        robot.moveLiftToPosition(0);
    }

    public void prepareToGrab () {
        robot.setPlacerUp();
        robot.moveGrabberToMid();
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
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
