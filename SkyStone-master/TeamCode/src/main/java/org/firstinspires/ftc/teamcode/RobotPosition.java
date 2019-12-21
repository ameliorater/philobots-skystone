package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class RobotPosition {
    DriveController dc;
    ModulePosition moduleLeftPosition;
    ModulePosition moduleRightPosition;
    double robotAbsXPos;
    double robotAbsYPos;
    double robotAbsHeading; //0 to 360 heading-style (clockwise is positive, 0 is straight ahead)

    final double ROBOT_WIDTH = 18*2.54; //in CM

    public RobotPosition (DriveController driveController, double startingXPos, double startingYPos) {
        this.dc = driveController;
        robotAbsXPos = startingXPos;
        robotAbsYPos = startingYPos;
        moduleLeftPosition = new ModulePosition(dc.moduleLeft);
        moduleRightPosition = new ModulePosition(dc.moduleRight);

    }

    public void update (Telemetry telemetry) {
        Vector2d rightDisp = moduleRightPosition.update(telemetry);
        Vector2d leftDisp = moduleLeftPosition.update(telemetry);

        rightDisp.setX(rightDisp.getX() + ROBOT_WIDTH/2);
        leftDisp.setX(leftDisp.getX() - ROBOT_WIDTH/2);

        Vector2d robotCenterDisp = new Vector2d((rightDisp.getX() + leftDisp.getX())/2, (rightDisp.getY() + leftDisp.getY())/2);
        robotCenterDisp.rotate(robotAbsHeading); //make field centric using previous heading
        robotAbsXPos += robotCenterDisp.getX();
        robotAbsYPos += robotCenterDisp.getY();

        Vector2d wheelToWheel = new Vector2d(rightDisp.getX() - leftDisp.getX(), rightDisp.getY() - leftDisp.getY()); //left to right
        //todo: check that get angle methods return correct things
        double robotAngleChange = wheelToWheel.getAngle();
        robotAbsHeading -= robotAngleChange; //minus because clockwise vs. counterclockwise (which one is positive changes)
        robotAbsHeading = robotAbsHeading % 360; //todo: check if we want this or the Python mod function
    }
}
