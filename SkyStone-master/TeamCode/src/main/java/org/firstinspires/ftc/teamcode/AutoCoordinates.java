package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;


@Autonomous(name = "AutoCoordinates")
public class AutoCoordinates extends LinearOpMode {
    //Robot class declaration
    double TILE_CM = 24 * 2.54;
    double ROBOT = 45;

    Robot robot;
    SkystoneCV cv;
    boolean isBlue = true; //todo: fix this

//    public AutoCoordinates () {
//
//    }

    public void runOpMode() {
        //Robot class initialization
        if (isBlue) {
            robot = new Robot(this, new Position(-83.46, 160.38, Angle.BACKWARD), true, true);
        } else {
            robot = new Robot(this, new Position(-83.46, -160.38, Angle.BACKWARD), true, true);
        }
        robot.initIMU();
        cv = new SkystoneCV("Webcam 1", new Point(15, 130), new Point(85, 130), new Point(155, 130), this);

        cv.init(SkystoneCV.CameraType.WEBCAM);
        waitForStart();
        //Autonomous actions
        //get Skystone position from vision class
        SkystoneCV.StonePosition skyStonePosition = cv.getSkystonePosition();
        //vision done
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();
        if(skyStonePosition == SkystoneCV.StonePosition.LEFT)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT), isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.BACKWARD), true, this);
        else if (skyStonePosition == SkystoneCV.StonePosition.CENTER)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT) - 8/*next stone val*/, isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.BACKWARD), true, this);
        else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT) - 16/*next stone val*/, isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.BACKWARD), true,this);
        intakeSequence();
    }
    public void intakeSequence(){
        //Todo:
        //kevin up and hold
        //intake on (fast mode)
        //hh down
        //hh up
        //drive forward
        //intake off
        //kevin down

    }
}
