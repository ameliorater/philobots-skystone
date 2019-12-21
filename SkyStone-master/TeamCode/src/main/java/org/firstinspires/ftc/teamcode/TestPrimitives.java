package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Primitives", group="Linear Opmode")

public class TestPrimitives extends LinearOpMode {
    Robot robot;

    public void runOpMode () {
        robot = new Robot(this, true);
        robot.initIMU();

        while(!isStarted()) {

            telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
            telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());
            telemetry.update();

            //throttle to 10Hz loop to avoid burning excess CPU cycles for no reason
            sleep(100);
        }



        robot.driveController.drive(Vector2d.FORWARD, 50, .7, this);
        robot.wait(1000, this);
        robot.driveController.rotateRobot(Angle.RIGHT, this);

//        while (opModeIsActive()) {
//            if (gamepad1.y) {
//                robot.driveController.rotateModules(Vector2d.FORWARD, 4000, this);
//            } else if (gamepad1.x) {
//                //drive 30 cm to the right (while facing forward)
//                robot.driveController.drive(Vector2d.FORWARD, 20, 1, this);
//            } else if (gamepad1.a) {
//                //turn to face robot right
//                robot.driveController.rotateRobot(Angle.RIGHT,this);
//            }
//        }
    }
}
