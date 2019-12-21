package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class FoundationAuto extends LinearOpMode {

    Robot robot;
    double DEFAULT_SPEED = 1;
    double TILE_CM = 24 * 2.54;

    boolean isBlue;

    FoundationAuto(boolean isBlue) {
        this.isBlue = isBlue;
    }

    public double toCm(double inches) { return inches * 2.54; }

    public void drive(Vector2d direction, double distance) {
        robot.driveController.drive(direction, distance, DEFAULT_SPEED, true, true, this);
    }

    public void rotate(Angle angle) {
        robot.driveController.rotateRobot(angle, this);
    }

    public void runOpMode() {

        robot = new Robot(this, true);
        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }

            //drive(Vector2d.FORWARD, toCm(48));
        robot.driveController.drive(Vector2d.FORWARD, TILE_CM*0.75, DEFAULT_SPEED, this);
        robot.driveController.drive(isBlue ? Vector2d.LEFT : Vector2d.RIGHT, TILE_CM*0.5 + (4*2.54), DEFAULT_SPEED, this);
        robot.driveController.rotateRobot(Angle.BACKWARD, this);
        robot.driveController.driveWithRange(Vector2d.FORWARD, 3, true, false, 0.3, 2000, this);
        /*
        //robot.driveController.drive(Vector2d.FORWARD, toCm(48), default_speed, this);
        robot.latch();
        robot.wait(4000, this);
        drive(Vector2d.BACKWARD, toCm(15));
        //robot.driveController.drive(Vector2d.BACKWARD, toCm(15), default_speed, this);
        rotate(Angle.LEFT);
        //robot.driveController.rotateRobot(Angle.LEFT, this);
        drive(Vector2d.RIGHT, toCm(20));
        //robot.driveController.drive(Vector2d.RIGHT, toCm(20), default_speed, this);
        robot.unlatch();
        robot.wait(4000, this);
        drive(Vector2d.FORWARD, toCm(10));
        //robot.driveController.drive(Vector2d.FORWARD, toCm(10), default_speed, this);
        drive(Vector2d.LEFT, toCm(48));
        //robot.driveController.drive(Vector2d.LEFT, toCm(48), default_speed, this);
        */

        robot.latch();
        robot.wait(2000, this);
        robot.driveController.drive(Vector2d.BACKWARD, TILE_CM*2.4, DEFAULT_SPEED, this);
        robot.unlatch();
        //robot.driveController.drive(isBlue ? Vector2d.RIGHT : Vector2d.LEFT, TILE_CM*3.2, DEFAULT_SPEED, this);
        robot.driveController.drive(isBlue ? Vector2d.RIGHT : Vector2d.LEFT, TILE_CM*3.25, DEFAULT_SPEED, this); //was *2
    }

}
