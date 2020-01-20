package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;


@Autonomous(name = "AutoCoordinates")
public class AutoCoordinates extends LinearOpMode {
    //Robot class declaration
    double Tile = 24 * 2.54;
    double ROBOT = 45;
// raw angle for move to, angle class get angle pass type 0-360 heading
    Robot robot;
    SkystoneCV cv;
    boolean isBlue = true;
    double currentPosX = 0;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    public void runOpMode() {
        //Robot class initialization
        robot = new Robot(this, true);
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
        simpleTracking.setPosition(-90, isBlue ? 157.5 : -157.5);
        simpleTracking.setOrientationDegrees(isBlue ? 180 : 0);
        robot.intakeServo1.setPosition(0.0);
        robot.intakeServo2.setPosition(1.0);
        robot.openGrabber();


        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130), this);


        cv.init(SkystoneCV.CameraType.WEBCAM);

        //Autonomous actions
        //get Skystone position from vision class
        SkystoneCV.StonePosition skyStonePosition = cv.getSkystonePosition();
        //vision done
        telemetry.addData("Skystone Pos: ", cv.getSkystonePosition());
        telemetry.update();
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();
        //todo the red vals may have + and - operators switched

        if(/*skyStonePosition == SkystoneCV.StonePosition.LEFT */ true){
            moveTo(-Tile - (.5 * ROBOT), isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), isBlue ? 180 : 0);
            simplePathFollow.stop(robot);
            currentPosX = -Tile - (.5 * ROBOT);
            intakeSequence();
        }
        else if (skyStonePosition == SkystoneCV.StonePosition.CENTER){
            moveTo(-Tile - (.5 * ROBOT) - 8/*next stone val*/, isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), isBlue ? 180 : 0);
            simplePathFollow.stop(robot);
            currentPosX = -Tile - (.5 * ROBOT) - 8;
            intakeSequence();
        }
        else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT){
            moveTo(-Tile - (.5 * ROBOT) - 16/*next stone val*/, isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), isBlue ? 180 : 0);
            simplePathFollow.stop(robot);
            currentPosX = -Tile - (.5 * ROBOT) - 16;
            intakeSequence();
        }
        robot.wait(1500, this);
        //move backwards to prepare for bridge crossing
        moveTo(-Tile, isBlue ? (1.5 * Tile): -(1.5 * Tile), isBlue ? 180 : 0);

        //rotate (avoid strafing entire bridge crossing)
        moveTo(-Tile, isBlue ? (1.5 * Tile): -(1.5 * Tile), 270);

        //drive under bridge
        moveTo((2 * Tile), isBlue ? (1.5 * Tile): -(1.5 * Tile), 270);
        // outtake block (intake reverse)

        //todo: vuforia read
        //rotate to face foundation
        moveTo((2 * Tile), isBlue ? (1.5 * Tile): -(1.5 * Tile), isBlue ? 0 : 180);

        //drive up to foundation
        moveWithRange((2 * Tile), isBlue ? (Tile) - ((ROBOT) / 2) : -((Tile) - ((ROBOT) / 2)), isBlue ? 0 : 180, 0.5, 5, 3, false);
        simplePathFollow.stop(robot);

        //spit out block (just in case)
        robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.FAST);

        //latch onto foundation
        robot.latch();

        //rotate foundation
        moveTo((2 * Tile), isBlue ? (Tile) - ((ROBOT) / 2) : -((Tile) - ((ROBOT) / 2)), 270);

        //moveTo((2 * Tile), isBlue ? (Tile * 2) : -((Tile * 2)), 270);

        //push foundation
        moveTo((3 * Tile) - (.5 * ROBOT), isBlue ? (Tile) - ((ROBOT) / 2) : -((Tile) - ((ROBOT) / 2)), 270, .7, 5, 1000);


        //drive robot into wall
        //moveTo((2 * Tile) + (Tile - ROBOT), isBlue ? (Tile * 2) : -((Tile * 2)), 270);

        //
        moveTo((3 * Tile) - (.5 * ROBOT), isBlue ? (Tile * 3) - (ROBOT / 2) : -((Tile * 3) - (ROBOT / 2)), 270, 0.5, 5, 3000); // this is a wall slam
        robot.setArmPower(1);
        robot.unlatch();
        //todo reset postion tracking for foundation in corner and against wall
        simpleTracking.setPosition(((Tile * 3) - 47) - (ROBOT / 2), (Tile * 3) - (ROBOT / 2));
        //todo get time remaining and decide weather to park or go for second


        moveTo((2 * Tile) + (Tile - ROBOT), isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), 270); // this is a wall slam
        moveTo(0, isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), 270);
        moveTo(-((Tile * 3) - (.5 * ROBOT)), isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), 270);
        moveTo(-((Tile * 3) - (.5 * ROBOT)), isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), isBlue ? 180 : 0);
        if(/*skyStonePosition == SkystoneCV.StonePosition.LEFT */ true){
            moveTo(-((Tile * 3) - (.5 * ROBOT)) + 16, isBlue ? (Tile) + ((ROBOT) / 2) : -((Tile) + ((ROBOT) / 2)), isBlue ? 180 : 0);
            currentPosX = -((Tile * 3) - (.5 * ROBOT)) + 16;
            intakeSequence();
        }
        else if (skyStonePosition == SkystoneCV.StonePosition.CENTER){
            moveTo(-((Tile * 3) - (.5 * ROBOT)) + 8/*next stone val*/, isBlue ? (Tile) + ((ROBOT) / 2) : -((Tile) + ((ROBOT) / 2)), isBlue ? 180 : 0);
            currentPosX = -((Tile * 3) - (.5 * ROBOT)) + 8;
            intakeSequence();
        }
        else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT){
            moveTo(-((Tile * 3) - (.5 * ROBOT))/*next stone val*/, isBlue ? (Tile) + ((ROBOT) / 2) : -((Tile) + ((ROBOT) / 2)), isBlue ? 180 : 0);
            currentPosX = -((Tile * 3) - (.5 * ROBOT));
            intakeSequence();
        }
        moveTo(currentPosX, isBlue ? (1.5 * Tile): -((1.5 * Tile)), isBlue ? 180 : 0);
        moveTo(currentPosX, isBlue ? (1.5 * Tile): -((1.5 * Tile)), 90);
        moveTo((Tile * 2) - (ROBOT * .5), isBlue ? (1.5 * Tile): -((1.5 * Tile)), 90);
        //todo arm out
        moveTo(0, isBlue ? (1.5 * Tile): -((1.5 * Tile)), 90);
    }


    public void intakeSequence(){
        //Todo: also add ifs for different blocks
        //kevin up .4 & .6
        robot.armServo1.setPosition(.4);
        robot.armServo2.setPosition(.6);
        //hh down
        robot.hungryHippoExtend();
        //intake on (slow)
        robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.SLOW);
        //wait 1500
        robot.wait(1500, this);
        //drive forward
        moveTo(currentPosX, isBlue ? Tile : -Tile, isBlue ? 180 : 0);
        simplePathFollow.stop(robot);
        //arm .3 and .7
        robot.armServo1.setPosition(.4);
        robot.armServo2.setPosition(.6);
        //hh up
        robot.hungryHippoRetract();
        //intake off //may need adjustment for double intake
        robot.moveIntake(Constants.IntakeState.STOP);
        //arm down
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);

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
    private void moveTo(double x, double y, double orientation) {
        moveTo(x, y, orientation, 0.7, 5);
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

    //uses position as far limit, will stop if range sees something before position is reached
    private void moveWithRange(double x, double y, double orientation, double speed, double threshold, double distanceToStop, boolean useFrontSensor) {
        boolean done = false;
        while (!done && opModeIsActive()) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            if (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold) {
                done = true;
            }
            if (robot.getRange(useFrontSensor) < distanceToStop) {
                done = true;
            }
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
