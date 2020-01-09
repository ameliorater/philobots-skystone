package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.opencv.core.Point;

public class Auto extends LinearOpMode {
    boolean isBlue;

    enum AutoType {BUILDING, LOADING, PARK}

    AutoType type;
    boolean mr;
    double DEFAULT_SPEED = 1; //was 0.8
    double SLOW_SPEED = 0.5; //was 0.3

    //helpful things
    double TILE_CM = 24 * 2.54;
    double ROBOT_CM = 18 * 2.54;

    public Auto(boolean isBlue, boolean isBuildingZone) {
        this.isBlue = isBlue;
        if (isBuildingZone) {
            type = AutoType.BUILDING;
        } else {
            type = AutoType.LOADING;
        }
    }

    public Auto(boolean moveRight) {
        type = AutoType.PARK;
        mr = moveRight;
    }

    Robot robot;
    SkystoneCV cv;
    Constants.IntakeSpeed INTAKE_SPEED = Constants.IntakeSpeed.FAST; //was fast
    double ARM_HOLDING_POWER = 0.1;

    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();
        if (isBlue)
            cv = new SkystoneCV("Webcam 1", new Point(205, 130), new Point(55, 130), new Point(135, 130), this);
        //else, if !isBlue
        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130), this);

        cv.init(SkystoneCV.CameraType.WEBCAM);

        while (!isStarted()) {
            cv.setWindows(gamepad2);
            cv.detector.printWindows(this);
            telemetry.addData("Skystone location: ", cv.getSkystonePosition());
            telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
            telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
            telemetry.update();

            //cheat code
            if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y)
                robot.driveController.resetEncoders();

            //throttle to 10Hz loop to avoid burning excess CPU cycles for no reason
            sleep(100);
        }

        //get Skystone position from vision class
        SkystoneCV.StonePosition skyStonePosition = cv.getSkystonePosition();
        //vision done
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();

        if (type == AutoType.BUILDING) {
            double SKYSTONE_DISTANCE;
            if (isBlue) {
                if (skyStonePosition == SkystoneCV.StonePosition.LEFT) {
                    //do nothing, already aligned
                    //SKYSTONE_DISTANCE = (8) * 2.54;
                    //SKYSTONE_DISTANCE = -8 + 7;
                    SKYSTONE_DISTANCE = -2;
                } else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT) {
                    //SKYSTONE_DISTANCE = (16 + 8) * 2.54;
                    SKYSTONE_DISTANCE = (8 + 7) * 2.54;
                } else { //defaults to CENTER
                    //SKYSTONE_DISTANCE = (8 + 8) * 2.54;
                    SKYSTONE_DISTANCE = (0 + 7) * 2.54;
                }
            } else {
                if (skyStonePosition == SkystoneCV.StonePosition.LEFT) {
                    //do nothing, already aligned
                    SKYSTONE_DISTANCE = (-8 - 7) * 2.54;
                } else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT) {
                    //SKYSTONE_DISTANCE = (8 - 7) * 2.54;
                    SKYSTONE_DISTANCE = 2 * 2.54;
                } else { //defaults to CENTER
                    SKYSTONE_DISTANCE = (0 - 7) * 2.54;
                }
            }

            if (SKYSTONE_DISTANCE > 0) {

            } else if (SKYSTONE_DISTANCE < 0) {

            }



        }

        // turn off any motors
        robot.moveIntake(Constants.IntakeState.STOP);
        // stop drive motors?

    }

    public void intakeSequence() {
        //deploy hungry hippo
        robot.hungryHippoExtend();
        robot.hungryHippoRetract();

        //intake block
        robot.moveIntake(Constants.IntakeState.INTAKE, INTAKE_SPEED);
        robot.driveController.drive(Vector2d.FORWARD, 20, SLOW_SPEED, this);
        //robot.wait(2000, this);
        robot.moveIntake(Constants.IntakeState.STOP);

        //backup until center of robot is 2in past center of tile
        //robot.driveController.driveWithRange(Vector2d.FORWARD, (TILE_CM/2)-(ROBOT_CM/2) + (2 * 2.54), false, true,SLOW_SPEED, this);
    }

    public void deliverBlock() {
        robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.FAST);
        robot.wait(1500, this);
        robot.moveIntake(Constants.IntakeState.STOP);

        //TODO: fix these numbers (idk what they need to be)
        //deliver block to foundation
        //while (robot.deployOuttake(0, 0)) {}
        //while (robot.returnOuttake(0, 0)) {}
    }

    public void waitForButton(Gamepad gamepad) {
//        while (!gamepad.b && opModeIsActive()) {
//            telemetry.addData("Waiting for button press", "");
//            telemetry.update();
//        }
//
//        while (gamepad1.b && opModeIsActive()) {
//            telemetry.addData("Waiting for button release", "");
//            telemetry.update();
//        }
    }

    public void moveLift (double power, double millis, boolean holdAfter) {
        robot.setArmPower(power); //was 0.3
        //wait
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && opModeIsActive()) {
        }
        if (holdAfter) robot.setArmPower(-ARM_HOLDING_POWER);
        else robot.setArmPower(0);
    }
}
