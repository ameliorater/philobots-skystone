package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Skystone TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    //public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    public boolean willResetIMU = true;

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)
//
//    public boolean hungryHippoDown = false;
//    public boolean foundationGrabberDown = false;
//    public double liftPower = 0;
//    public boolean intakeOn = false;

    boolean toDeploy;
    boolean toRetract;

    double lastTime;

    Vector2d joystick1, joystick2;

    public void init() {
        robot = new Robot(this, false, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
        lastTime = getRuntime();
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();
        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;

        telemetry.addData("Robot Heading: ", robot.getRobotHeading());
        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

        //slow mode/range stuffs
        if (gamepad1.left_trigger > 0.1) {
            joystick1 = joystick1.scale(0.3);
            joystick2 = joystick2.scale(0.4); //was 0.3
            slowModeDrive = true;
        }
        else if (gamepad1.right_trigger > 0.1) {
            if (robot.getRange(false) < 30) {
                joystick1 = joystick1.scale(0.3);
                joystick2 = joystick2.scale(0.4); //was 0.3
                slowModeDrive = true;
            }
        }
        else if (gamepad1.right_bumper) {
            robot.unlatch();
        } else if (gamepad1.left_bumper) {
            robot.latch();
        }

//        //toggle abs heading
//        if (gamepad1.y) {
//            absHeadingMode = true;
//        }
//        if (gamepad1.b) {
//            absHeadingMode = false;
//        }

        //marker
        if (gamepad1.y && gamepad1.b) {
            robot.deployMarker();
        }
        if (gamepad1.y && gamepad1.x) {
            robot.retractMarker();
        }

        robot.driveController.updateUsingJoysticks(
                checkDeadband(joystick1, slowModeDrive).scale(Math.sqrt(2)),
                checkDeadband(joystick2, slowModeDrive).scale(Math.sqrt(2)),
                absHeadingMode
        );

//        if (gamepad2.dpad_up) {
//            //robot.hungryHippoExtend();
//            robot.moveServo(robot.backStop, 1);
//        } else if (gamepad2.dpad_down) {
//            //robot.hungryHippoRetract();
//            robot.moveServo(robot.backStop, 0);
//        }

        /*
        if (gamepad2.dpad_right) {
            robot.intakeServoOpen();
        } else if (gamepad2.dpad_left) {
            robot.intakeServoClose();
        }
        */

        // Changed button map - see below
//        if (gamepad2.y) {
//            robot.unlatch();
//        } else if (gamepad2.a) {
//            robot.latch();
//        }

        if (gamepad1.dpad_up) {
            robot.unlatch();
        } else if (gamepad1.dpad_down) {
            robot.latch();
        }

        if ((gamepad2.x) && (gamepad2.right_trigger > 0.1)) {
            robot.moveGrabberToMid();
        } else if (gamepad2.x) {
            robot.openGrabber();
        } else if (gamepad2.b) {
            robot.closeGrabber();
        }

        int x_input, y_input;
        if(Math.abs(gamepad2.right_stick_x) < 0.05) //dead band
            x_input = 0;
        else x_input = (int)gamepad2.right_stick_x;

        if(Math.abs(gamepad2.right_stick_y) < 0.05)
            y_input = 0;
        else y_input = (int)gamepad2.right_stick_y;


        //Manual control overrides automated deploy and retract
//        if(x_input != 0 || y_input != 0){
//            toDeploy = false;
//            toRetract = false;
//        }
//        else if(/*gamepad2.right_bumper*/gamepad2.y) {
//            toDeploy = true;
//            toRetract = false;
//        }
//        else if(/*gamepad2.left_bumper*/gamepad2.a){
//            toRetract = true;
//            toDeploy = false;
//        }
//
//        robot.moveOuttake(x_input, y_input, toDeploy, toRetract);

        double currentTime = getRuntime();
        if (gamepad2.a) {
            robot.currentClawPosition.moveSequence(robot.controller.INSIDE_ROBOT_TO_DELIVERY, currentTime - lastTime);
            //robot.backStop.setPosition(0);
        }
        if (gamepad2.y) {
            //robot.closeGrabber();
            robot.currentClawPosition.moveSequence(robot.controller.DELIVERY_TO_INSIDE_ROBOT, currentTime - lastTime);
// don't close grabber for the side grabber
//            robot.grabberServo.setPosition(0.5);
            //robot.backStop.setPosition(1);
        }
//        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
//            robot.currentClawPosition.moveBy(gamepad2.right_stick_x, 0, currentTime - lastTime);
//        }

        // Adding x-y control to claw ouside of robot
        //        if (gamepad2.dpad_up) {
//            //robot.hungryHippoExtend();
//            robot.moveServo(robot.backStop, 1);
//        } else if (gamepad2.dpad_down) {
//            //robot.hungryHippoRetract();
//            robot.moveServo(robot.backStop, 0);
//        }

        double clawDeltaX = 0;
        double clawDeltaY = 0;
        if (gamepad2.dpad_up)  clawDeltaY = 1;
        if (gamepad2.dpad_down) clawDeltaY = -1;
        if (gamepad2.dpad_left) clawDeltaX = -1;
        if (gamepad2.dpad_right) clawDeltaX = 1;

        if ((clawDeltaX != 0) || (clawDeltaY != 0)) {
            robot.currentClawPosition.moveBy(clawDeltaX, clawDeltaY, currentTime - lastTime);
        }
        // moved the backstop to the right stick to make room for the x-y claw controls
//        if (gamepad2.right_stick_x > 0.1) {
//            robot.moveServo(robot.placer, 1);
//        } else { //was else if (gamepad2.right_stick_x < -0.1)
//            robot.moveServo(robot.placer, 0);
//        }


        robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
        robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);

        final double ROTATE_SPEED = 180.0/270;
        robot.outatkeRotatePosition += gamepad2.right_stick_y * ROTATE_SPEED * (currentTime - lastTime);
        if (robot.outatkeRotatePosition > 1) robot.outatkeRotatePosition = 1;
        if (robot.outatkeRotatePosition < 0) robot.outatkeRotatePosition = 0;
        robot.outtakeRot.setPosition(robot.outatkeRotatePosition);

        lastTime = currentTime;

        /*
        if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.FAST);
        } else if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.FAST);
        } else if (gamepad2.right_bumper) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.SLOW);
        } else if (gamepad2.left_bumper) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.SLOW);
        } else {
            robot.moveIntake(Constants.IntakeState.STOP, Constants.IntakeSpeed.STOPPED);
        }*/


        if (gamepad1.dpad_up) {
            robot.hungryHippoExtend();
        } else if (gamepad1.dpad_down) {
            robot.hungryHippoRetract();
        }

        if (gamepad1.dpad_left) {
            robot.moveSingleIntakeRoller(true);
        } else if (gamepad1.dpad_right) {
            robot.moveSingleIntakeRoller(false);
        } else if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.SLOW); //was fast
        } else if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.SLOW); //was fast
        } /*else if (gamepad2.right_bumper) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.SLOW);
        } else if (gamepad2.left_bumper) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.SLOW);
        }*/ else {
            robot.moveIntake(Constants.IntakeState.STOP, Constants.IntakeSpeed.STOPPED);
        }

        //robot.setArmPower(-gamepad2.right_stick_y);
        telemetry.addData("Arm power", -gamepad2.right_stick_x);
        robot.moveLift(-gamepad2.left_stick_y);

        //SCARA arm back position centered on x: (-76, -185); out of robot: (-76, 150)

        if (gamepad2.right_bumper) {
            robot.moveServo(robot.placer, 1); // move down
        } else if (gamepad2.left_bumper) {
            robot.moveServo(robot.placer, 0); // move up
        }
// moved placer to gamepad2's bumpers
//        if (gamepad2.right_bumper) {
//            robot.moveLift(Constants.SLOW_LIFT_POWER_UP);
//        } else if (gamepad2.left_bumper) {
//            robot.moveLift(Constants.SLOW_LIFT_POWER_DOWN);
//        } else {
//            robot.moveLift(-gamepad2.left_stick_y);
//        }

        //todo: remove after done tuning
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//
//        if (gamepad2.dpad_right) {
//            robot.LIFT_TICKS_PER_MS += 0.05;
//        } else if (gamepad2.dpad_left) {
//            robot.LIFT_TICKS_PER_MS -= 0.05;
//        }
//
//        if (gamepad1.right_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG += 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG += 0.01;
//        }
//
//        if (gamepad1.left_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG -= 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG -= 0.01;
//        }

        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        telemetry.addData("joystick 1", joystick1);
        telemetry.addData("joystick 2", joystick2);

        loopEndTime = System.currentTimeMillis();
        telemetry.addData("Our loop time: ", loopEndTime - loopStartTime);

        telemetry.update();
    }

    public void stop() {
        robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
        super.stop();
    }



    /*//This implementation caused a bug - vectors that go in any cardinal direction would get reduced to a point vector, for example
    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return new Vector2d(0, 0);
    }*/

    // Instead of using the above implementation, this one checks
    // if the length of the vector is more than that of the deadband magnitude
    public Vector2d checkDeadband(Vector2d joystick, boolean slowMode) {
        return  joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL) ?
                joystick : new Vector2d(0, 0);
    }
}