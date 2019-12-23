package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Skystone TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    public boolean willResetIMU = true;

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


    public void init() {
        robot = new Robot(this, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }

    public void loop() {
        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            joystick1 = joystick1.scale(0.3);
            joystick2 = joystick2.scale(0.3);
        }

        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1).scale(Math.sqrt(2)), checkDeadband(joystick2).scale(Math.sqrt(2)));

        if (gamepad2.dpad_up) {
            robot.hungryHippoExtend();
        } else if (gamepad2.dpad_down) {
            robot.hungryHippoRetract();
        }

        if (gamepad2.y) {
            robot.unlatch();
        } else if (gamepad2.a) {
            robot.latch();
        }

        if (gamepad2.x) {
            robot.openGrabber();
        } else if (gamepad2.b) {
            robot.closeGrabber();
        }

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


        if (gamepad1.dpad_left) {
            robot.moveSingleIntakeRoller(true);
        } else if (gamepad1.dpad_right) {
            robot.moveSingleIntakeRoller(false);
        } else if (Math.abs(gamepad2.right_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.FAST);
        } else if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.FAST);
        } else if (gamepad2.right_bumper) {
            robot.moveIntake(Constants.IntakeState.INTAKE, Constants.IntakeSpeed.SLOW);
        } else if (gamepad2.left_bumper) {
            robot.moveIntake(Constants.IntakeState.OUTTAKE, Constants.IntakeSpeed.SLOW);
        } else {
            robot.moveIntake(Constants.IntakeState.STOP, Constants.IntakeSpeed.STOPPED);
        }

        robot.setArmPower(-gamepad2.right_stick_y);
        telemetry.addData("Arm power", -gamepad2.right_stick_x);
        robot.moveLift(-gamepad2.left_stick_y);


        //remove after done tuning
        if (gamepad1.b) {
            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
        }
        if (gamepad1.x) {
            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
        }

        if (gamepad2.dpad_right) {
            robot.LIFT_TICKS_PER_MS += 0.05;
        } else if (gamepad2.dpad_left) {
            robot.LIFT_TICKS_PER_MS -= 0.05;
        }
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
        telemetry.update();
    }

    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return new Vector2d(0, 0);
    }
}