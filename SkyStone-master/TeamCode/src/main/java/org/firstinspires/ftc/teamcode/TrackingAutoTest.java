package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Tracking Test", group="Linear Opmode")
public class TrackingAutoTest extends LinearOpMode {

    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;

    public void runOpMode() {

        boolean isBlue = true;

        robot = new Robot(this, new Position(-90, 157.5, Angle.FORWARD), true, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // optionally set starting position and orientation of the robot
        simpleTracking.setOrientationDegrees(0);
        simpleTracking.setPosition(0,0);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        simpleTracking.setModuleOrientation(robot);

        // position intake wheels inside the robot
        robot.intakeServo1.setPosition(1.0);
        robot.intakeServo2.setPosition(0.0);


        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        // blue starting position
        simpleTracking.setPosition(-90, 157.5);
        simpleTracking.setOrientationDegrees(180);
        robot.intakeServo1.setPosition(0.0);
        robot.intakeServo2.setPosition(1.0);
        robot.openGrabber();

        double stonePosition = -98;// depends on stone
        // go to the center stone
        robot.driveController.driveToPosition(new Position(stonePosition, 85, Angle.FORWARD), isBlue, this);
        simplePathFollow.stop(robot);
        robot.armServo1.setPosition(.4);
        robot.armServo2.setPosition(.6);
        robot.hungryHippoRetract();
        robot.moveIntake(Constants.IntakeState.INTAKE,Constants.IntakeSpeed.SLOW);
        robot.wait(1500, this);
        robot.driveController.driveToPosition(new Position(stonePosition, 62.5, Angle.FORWARD), true, this);
        simplePathFollow.stop(robot);
        robot.armServo1.setPosition(.7);
        robot.armServo2.setPosition(.3);
        robot.wait(1000,this);
        robot.hungryHippoExtend();
        robot.moveIntake(Constants.IntakeState.STOP);
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);

        robot.wait(1000, this);
        robot.driveController.driveToPosition(new Position(stonePosition, 95, Angle.FORWARD), true, this);
        robot.driveController.driveToPosition(new Position(stonePosition, 95, Angle.RIGHT), isBlue, this);
        simplePathFollow.stop(robot);
        robot.wait(1000, this);
        //moveTo(-90, 100, 270, 0.6, 5);
        //simplePathFollow.stop(robot);


        // move to platform
        robot.driveController.driveToPosition(new Position(120, 95, Angle.RIGHT), isBlue, this);
        robot.driveController.driveToPosition(new Position(120, 90, Angle.BACKWARD), isBlue, this);
        simplePathFollow.stop(robot);
        robot.driveController.driveToPosition(new Position(120, 55, Angle.BACKWARD), isBlue, this);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);
        robot.wait(200, this);
        robot.closeGrabber();
        robot.wait(200, this);
        robot.openGrabber();
        robot.wait(200, this);
        robot.closeGrabber();
        robot.wait(500, this);
        robot.armServo1.setPosition(.3);
        robot.armServo2.setPosition(.7);
        robot.wait(1200, this);
        robot.openGrabber();
        robot.armServo1.setPosition(.7);
        robot.armServo2.setPosition(.3);

        robot.wait(1000, this);
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);
        robot.driveController.driveToPosition(new Position(120, 115, Angle.BACKWARD), isBlue, this);
        simplePathFollow.stop(robot);
        robot.driveController.driveToPosition(new Position(80, 115, new Angle(65, Angle.AngleType.ZERO_TO_360_HEADING)), isBlue, this); //pivot platform todo: fix 245
        robot.driveController.driveToPosition(new Position(110, 115, new Angle(90, Angle.AngleType.ZERO_TO_360_HEADING)), isBlue, this);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);
        robot.driveController.driveToPosition(new Position(60, 90, Angle.RIGHT), isBlue, this);
        robot.driveController.driveToPosition(new Position(0, 90, Angle.RIGHT), isBlue, this);
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

    //move relative to the current position
    private void move(double x, double y, double orientation, double speed, double threshold) {
        moveTo(simpleTracking.lastRobotX + x, simpleTracking.lastRobotY + y, simpleTracking.lastRobotOrientation + orientation, speed, threshold);
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
