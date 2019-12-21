package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Vector;

public class ModulePosition {

    DriveModule module;

    double lastM1Encoder;
    double lastM2Encoder;

    public ModulePosition(DriveModule driveModule) {
        this.module = driveModule;
        lastM1Encoder = module.motor1.getCurrentPosition();
        lastM2Encoder = module.motor1.getCurrentPosition();
    }

    public Vector2d update (Telemetry telemetry) {
        double newM1Encoder = module.motor1.getCurrentPosition();
        double newM2Encoder = module.motor2.getCurrentPosition();

        double startingAngle = (lastM1Encoder + lastM2Encoder)/2.0 * module.DEGREES_PER_TICK;
        double finalAngle = (newM1Encoder + newM2Encoder)/2.0 * module.DEGREES_PER_TICK;
        double angleChange = Math.toRadians(finalAngle - startingAngle);

        telemetry.addData("Starting angle: ", startingAngle);
        telemetry.addData("Final angle: ", finalAngle);
        telemetry.addData("Starting angle radians: ", Math.toRadians(startingAngle));
        telemetry.addData("Final angle radians: ", Math.toRadians(finalAngle));
        telemetry.addData("Angle change: ", angleChange);

        //todo: check subtraction order
        double startingPosition = (lastM1Encoder - lastM2Encoder);
        double finalPosition = (newM1Encoder - newM2Encoder);
        double positionChange = (finalPosition - startingPosition)/2.0 * module.CM_PER_TICK;

        telemetry.addData("Starting position: ", startingPosition);
        telemetry.addData("Final position: ", finalPosition);
        telemetry.addData("Position change: ", positionChange);

        Vector2d displacementVec;
        //derivation documented elsewhere
        if (angleChange != 0) {
            double deltaXPos = positionChange/angleChange * (Math.sin(Math.toRadians(startingAngle)) - Math.sin(Math.toRadians(finalAngle)));
            double deltaYPos = positionChange/angleChange * (Math.cos(Math.toRadians(finalAngle)) - Math.cos(Math.toRadians(startingAngle)));
            displacementVec = new Vector2d(deltaXPos, deltaYPos);
            telemetry.addData("Doing the fancy way ", displacementVec.getX());
        } else if (positionChange == 0) {
            telemetry.addData("Nothing is moving ", "");
            displacementVec = new Vector2d(0, 0);
        } else {
            displacementVec = new Vector2d(new Angle(startingAngle, Angle.AngleType.ZERO_TO_360_HEADING));
            displacementVec.normalize(positionChange);
            telemetry.addData("Doing the straight line way ","");
        }

        telemetry.addData("Delta X Pos: ", displacementVec.getX());
        telemetry.addData("Delta Y Pos: ", displacementVec.getY()); //was printing the final position instead...
        //telemetry.addData("Position change/angle change: ",positionChange/angleChange);


        lastM1Encoder = newM1Encoder;
        lastM2Encoder = newM2Encoder;

        return displacementVec;
    }

//    public void setAbsPosition (double absXPos, double absYPos) {
//        this.absXPos = absXPos;
//        this.absYPos = absYPos;
//    }

}