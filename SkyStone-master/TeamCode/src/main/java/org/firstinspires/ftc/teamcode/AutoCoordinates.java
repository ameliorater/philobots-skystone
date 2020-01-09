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
    public void runOpMode() {
        //Robot class initialization
        robot = new Robot(this, new Position(-83.46, 160.32, Angle.FORWARD), true, true);
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

        robot.intakeServo1.setPosition(1.0);
        robot.intakeServo2.setPosition(0.0);
        robot.openGrabber();

        if(skyStonePosition == SkystoneCV.StonePosition.LEFT)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT), isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.FORWARD), isBlue, this);
        else if (skyStonePosition == SkystoneCV.StonePosition.CENTER)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT) - 8/*next stone val*/, isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.FORWARD), isBlue, this);
        else if (skyStonePosition == SkystoneCV.StonePosition.RIGHT)
            robot.driveController.driveToPosition(new Position(-TILE_CM - (.5 * ROBOT) - 16/*next stone val*/, isBlue ? (3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT)) : -((3 * TILE_CM) - (.5 * ROBOT) - (TILE_CM + (TILE_CM - ROBOT))), Angle.FORWARD), isBlue,this);

        //intake sequence
        robot.armServo1.setPosition(.4);
        robot.armServo2.setPosition(.6);
        robot.hungryHippoRetract();
        robot.moveIntake(Constants.IntakeState.INTAKE,Constants.IntakeSpeed.SLOW);
        robot.wait(1500, this);
        //moveTo(stonePosition, 62.5, 180, 0.35, 5);
        //simplePathFollow.stop(robot);
        robot.armServo1.setPosition(.7);
        robot.armServo2.setPosition(.3);
        robot.wait(1000,this);
        robot.hungryHippoExtend();
        robot.moveIntake(Constants.IntakeState.STOP);
        robot.armServo1.setPosition(.5);
        robot.armServo2.setPosition(.5);

        robot.wait(1000, this);


        robot.driveController.driveToPosition(new Position (robot.driveController.robotPosition.x, robot.driveController.robotPosition.y, isBlue ? Angle.RIGHT : Angle.LEFT), isBlue, this);
        robot.driveController.driveToPosition(new Position(-TILE_CM, isBlue ? (2 * TILE_CM) - ((TILE_CM * ROBOT) / 2) : -((2 * TILE_CM) - ((TILE_CM * ROBOT) / 2)), Angle.RIGHT), isBlue,this);
        robot.driveController.driveToPosition(new Position((2 * TILE_CM), isBlue ? (2 * TILE_CM) - ((TILE_CM * ROBOT) / 2) : -((2 * TILE_CM) - ((TILE_CM * ROBOT) / 2)), Angle.RIGHT), isBlue,this);
        robot.driveController.driveToPosition(new Position (robot.driveController.robotPosition.x, robot.driveController.robotPosition.y, Angle.BACKWARD), isBlue, this);
        //todo: outtake block (intake reverse)
        //todo: vuforia read
        robot.driveController.driveToPosition(new Position (robot.driveController.robotPosition.x, robot.driveController.robotPosition.y, Angle.BACKWARD), isBlue, this);
        robot.driveController.driveToPosition(new Position((2 * TILE_CM), isBlue ? (TILE_CM) - ((ROBOT) / 2) : -((TILE_CM) - ((ROBOT) / 2)), Angle.BACKWARD), isBlue,this);
        //todo hooks down
        robot.driveController.driveToPosition(new Position (robot.driveController.robotPosition.x, robot.driveController.robotPosition.y, isBlue ? Angle.RIGHT : Angle.LEFT), isBlue, this);
        robot.driveController.driveToPosition(new Position((2 * TILE_CM), isBlue ? (TILE_CM * 2) : -((TILE_CM * 2)), Angle.RIGHT), isBlue,this);
        robot.driveController.driveToPosition(new Position((2 * TILE_CM) + (TILE_CM - ROBOT), isBlue ? (TILE_CM * 2) : -((TILE_CM * 2)), Angle.RIGHT), isBlue,this);
        robot.driveController.driveToPosition(new Position((2 * TILE_CM) + (TILE_CM - ROBOT), isBlue ? (TILE_CM * 3) - (ROBOT / 2) : -((TILE_CM * 3) - (ROBOT / 2)), Angle.RIGHT), isBlue,this); // this is a wall slam
        //todo arm delivers block
        //todo hooks up
        robot.driveController.driveToPosition(new Position((2 * TILE_CM) + (TILE_CM - ROBOT), isBlue ? (TILE_CM * 2) - ((TILE_CM - ROBOT) / 2) : -((TILE_CM * 2) - ((TILE_CM - ROBOT) / 2)), Angle.RIGHT), isBlue,this); // this is a wall slam
        robot.driveController.driveToPosition(new Position(0, isBlue ? (TILE_CM * 2) - ((TILE_CM - ROBOT) / 2) : -((TILE_CM * 2) - ((TILE_CM - ROBOT) / 2)), Angle.RIGHT), isBlue,this); // this is a wall slam

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
