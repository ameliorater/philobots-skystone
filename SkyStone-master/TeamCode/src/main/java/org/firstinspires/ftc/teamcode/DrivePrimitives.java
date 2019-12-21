package org.firstinspires.ftc.teamcode;

public class DrivePrimitives {
    Robot robot;
    DriveController dc;

    public DrivePrimitives (Robot robot) {
        this.robot = robot;
        dc = robot.driveController; //for easier reference
    }
}