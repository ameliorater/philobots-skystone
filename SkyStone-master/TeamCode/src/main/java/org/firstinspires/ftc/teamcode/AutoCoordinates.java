package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;


@Autonomous(name = "AutoCoordinates")
public class AutoCoordinates extends LinearOpMode {
    //Robot class declaration
    double Tile = 24 * 2.54;
    double ROBOT = 45;

    Robot robot;
    SkystoneCV cv;
    public void runOpMode() {
        //Robot class initialization
        robot = new Robot(this, true);
        robot.initIMU();
        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130), this);

        boolean isBlue = true;
        cv.init(SkystoneCV.CameraType.WEBCAM);
        waitForStart();
        //Autonomous actions
        //get Skystone position from vision class
        SkystoneCV.StonePosition skyStonePosition = cv.getSkystonePosition();
        //vision done
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();
        if(skyStonePosition == SkystoneCV.StonePosition.LEFT)
            robot.driveController.driveToPosition(new Position(-Tile - (.5 * ROBOT), isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), Angle.FORWARD), this);
        else if (skyStonePosition == SkystoneCV.StonePosition.CENTER)
            robot.driveController.driveToPosition(new Position(-Tile - (.5 * ROBOT) - 8/*next stone val*/, isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), Angle.FORWARD), this);
        else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT)
            robot.driveController.driveToPosition(new Position(-Tile - (.5 * ROBOT) - 16/*next stone val*/, isBlue ? (3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT)) : -((3 * Tile) - (.5 * ROBOT) - (Tile + (Tile - ROBOT))), Angle.FORWARD), this);
        intakeSequence();
        robot.driveController.driveToPosition(new Position(-Tile, isBlue ? (2 * Tile) - ((Tile * ROBOT) / 2) : -((2 * Tile) - ((Tile * ROBOT) / 2)), Angle.RIGHT), this);
        robot.driveController.driveToPosition(new Position((2 * Tile), isBlue ? (2 * Tile) - ((Tile * ROBOT) / 2) : -((2 * Tile) - ((Tile * ROBOT) / 2)), Angle.RIGHT), this);
        robot.driveController.rotateRobot(Angle.BACKWARD, this);
        //todo: outtake block (intake reverse)
        //todo: vuforia read
        robot.driveController.driveToPosition(new Position((2 * Tile), isBlue ? (Tile) - ((ROBOT) / 2) : -((Tile) - ((ROBOT) / 2)), Angle.BACKWARD), this);
        //todo hooks down
        robot.driveController.driveToPosition(new Position((2 * Tile), isBlue ? (Tile * 2) : -((Tile * 2)), Angle.RIGHT), this);
        robot.driveController.driveToPosition(new Position((2 * Tile) + (Tile - ROBOT), isBlue ? (Tile * 2) : -((Tile * 2)), Angle.RIGHT), this);
        robot.driveController.driveToPosition(new Position((2 * Tile) + (Tile - ROBOT), isBlue ? (Tile * 3) - (ROBOT / 2) : -((Tile * 3) - (ROBOT / 2)), Angle.RIGHT), this); // this is a wall slam
        //todo arm delivers block
        //todo hooks up
        robot.driveController.driveToPosition(new Position((2 * Tile) + (Tile - ROBOT), isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), Angle.RIGHT), this); // this is a wall slam
        robot.driveController.driveToPosition(new Position(0, isBlue ? (Tile * 2) - ((Tile - ROBOT) / 2) : -((Tile * 2) - ((Tile - ROBOT) / 2)), Angle.RIGHT), this); // this is a wall slam





    }
    public void intakeSequence(){
        //Todo:
        //kevin up and hold
        //intake on (fast mode)
        //robot.moveIntake();
        //hh down
        //hh up
        //drive forward
        //intake off
        //kevin down

    }
}
