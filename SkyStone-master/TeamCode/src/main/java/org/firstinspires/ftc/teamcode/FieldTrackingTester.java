package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

<<<<<<< HEAD
@Autonomous(name="FieldTrackingTestv2", group ="Auto")
=======
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="FieldTrackingTest", group ="Auto")
>>>>>>> 8a70bf03325c5210c08d5c0c270ac79ea921df32
public class FieldTrackingTester extends LinearOpMode {

    static FieldTracker tracker;

<<<<<<< HEAD
    public void runOpMode() {
        tracker = new FieldTracker(hardwareMap, telemetry);
=======
    public void setupIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }
>>>>>>> 8a70bf03325c5210c08d5c0c270ac79ea921df32

    public void runOpMode() {
        //CHANGE PARAMETERS FOR WEBCAM/GRAPHICS USAGE
        tracker = new FieldTracker(hardwareMap, telemetry, true, true);
        setupIMU();

<<<<<<< HEAD
        while (opModeIsActive() && !isStopRequested()) {
            tracker.logInfo();

=======
        double imuAngle, trackerAngle;

        waitForStart();

        long previousTime = System.currentTimeMillis(), currentTime;

        while (opModeIsActive()) {

            imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            TargetInfo trackerInfo = tracker.getTargetInfo();

            telemetry.addData("IMU Rotation", imuAngle);

            if (trackerInfo != null) {
                trackerAngle = trackerInfo.zRotation;
                telemetry.addData("Tracker Z Rotation", trackerAngle);
                telemetry.addData("Difference", imuAngle - trackerAngle);
            }

            telemetry.addData("\n\nTracker Info", tracker.getTargetInfo());

            currentTime = System.currentTimeMillis();
            telemetry.addData("Update speed", currentTime - previousTime + " milliseconds");
            previousTime = currentTime;

            telemetry.update();
>>>>>>> 8a70bf03325c5210c08d5c0c270ac79ea921df32
        }
    }

}
