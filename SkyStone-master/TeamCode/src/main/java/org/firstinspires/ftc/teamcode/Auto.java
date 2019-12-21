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

        //TWO SKYSTONES + FOUNDATION !!
        //assumes lineup in center of tile next to depot
        // need to be 18 cm to the right of block center to get it
        double CROSS_FIELD_DIST_CM = TILE_CM * 4.7; //was 4.6


        //notes
        //on blue, robot center needs to be to the LEFT of the block it's trying to get
        //  by about 7 inches, and on red it needs to be to the RIGHT

        if (type == AutoType.BUILDING) {
            //7*2.54 + 2.9*2.54 to the RIGHT for blue and to the LEFT for red
            if (opModeIsActive())
                robot.driveController.drive(Vector2d.FORWARD, 78, DEFAULT_SPEED, this); //was 80cm

            waitForButton(gamepad1);

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

            //BAAAD
            if (isBlue) SKYSTONE_DISTANCE += 1;

            if (SKYSTONE_DISTANCE > 0) {
                robot.driveController.drive(Vector2d.RIGHT, SKYSTONE_DISTANCE, DEFAULT_SPEED, this); //tile/3
            } else if (SKYSTONE_DISTANCE < 0) {
                robot.driveController.drive(Vector2d.LEFT, Math.abs(SKYSTONE_DISTANCE), DEFAULT_SPEED, this); //tile/3
            }

            waitForButton(gamepad1);

            if (isBlue)
                robot.driveController.rotateRobot(new Angle(-150, Angle.AngleType.NEG_180_TO_180_HEADING), this); //blue was +150
            else
                robot.driveController.rotateRobot(new Angle(150, Angle.AngleType.NEG_180_TO_180_HEADING), this);
            //turn on intake
            robot.moveIntake(Constants.IntakeState.INTAKE, INTAKE_SPEED);

            waitForButton(gamepad1);

            moveLift(-0.3, 400, true); //WAS 350

            //drive up close to blocks
            //robot.driveController.driveWithRange(Vector2d.FORWARD, 10, true, true, SLOW_SPEED, this);
            robot.driveController.drive(Vector2d.FORWARD, 90, SLOW_SPEED, this); //was 60cm

            robot.wait(750, this);
            //turn off intake
            robot.moveIntake(Constants.IntakeState.STOP);

            moveLift(0.3, 750, false);

            waitForButton(gamepad1);


            //backup
            robot.driveController.drive(Vector2d.BACKWARD, 95, SLOW_SPEED, this); //was 105

            //REMOVED
            waitForButton(gamepad1);
            //rotate to face forwards (needs to be straight to fit in slot)
            robot.driveController.rotateRobot(Angle.BACKWARD, 0.4,this);

            waitForButton(gamepad1);
            //drive towards foundation (stop on distance from wall)
            //robot.driveController.driveWithRange(isBlue ? Vector2d.LEFT : Vector2d.RIGHT, TILE_CM * 1.5, true, true, DEFAULT_SPEED, this);
            if (isBlue) robot.driveController.drive(isBlue ? Vector2d.LEFT : Vector2d.RIGHT, CROSS_FIELD_DIST_CM + SKYSTONE_DISTANCE + 120, DEFAULT_SPEED, this); //was +80
            else robot.driveController.drive(isBlue ? Vector2d.LEFT : Vector2d.RIGHT, CROSS_FIELD_DIST_CM - SKYSTONE_DISTANCE + 120, DEFAULT_SPEED, this); //was +80

            //robot.driveController.rotateRobot(Angle.BACKWARD, this);

//            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.FAST); //just in case
//            robot.wait(500, this);
//            robot.moveIntake(Constants.IntakeState.STOP); //just in case

            //robot.driveController.drive(isBlue ? Vector2d.LEFT : Vector2d.RIGHT, 80, DEFAULT_SPEED, this);

            //ADDED
            robot.driveController.rotateRobot(Angle.BACKWARD, this);

            //approach foundation
            robot.driveController.driveWithRange(Vector2d.FORWARD, 3, true, false, SLOW_SPEED, 2000, this);

            robot.closeGrabber();
            robot.latch();

            //deliver block
            moveLift(-0.3, 1000, false); //was 1200
            robot.openGrabber();
            robot.wait(500, this);
            moveLift(0.3, 1000, false); //was 1400

            robot.driveController.driveWithTimeout(Vector2d.BACKWARD, TILE_CM*2.6, DEFAULT_SPEED, 2500, this); //was 2.4

            robot.unlatch();
            //robot.driveController.drive(isBlue ? Vector2d.RIGHT : Vector2d.LEFT, TILE_CM*3.2, DEFAULT_SPEED, this);
            robot.driveController.drive(isBlue ? Vector2d.RIGHT : Vector2d.LEFT, TILE_CM*2, DEFAULT_SPEED, this); //was 1.7
            robot.driveController.drive(Vector2d.FORWARD, TILE_CM*1, DEFAULT_SPEED, this);
            robot.driveController.drive(isBlue ? Vector2d.RIGHT : Vector2d.LEFT, TILE_CM*.8, DEFAULT_SPEED, this); //WAS 0.5


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
