package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Drive To Position Tester", group = "Auto")
public class DriveToPositionTester extends LinearOpMode {
    //Robot class declaration
    Robot robot;

    public void runOpMode() {
        //Robot class initialization
        robot = new Robot(this, true, true);
        robot.initIMU();
        robot.driveController.resetEncoders();
        while (!isStarted()) {
            telemetry.addData("Ready to start", "");
            telemetry.update();
        }

        //Autonomous actions
        robot.driveController.driveToPosition(
                new Position(0, 120, new Angle(90, Angle.AngleType.ZERO_TO_360_HEADING)), this);

        while (opModeIsActive()) {}
    }
}

