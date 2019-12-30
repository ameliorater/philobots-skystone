package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="FieldTrackingTestv2", group ="Auto")
public class FieldTrackingTester extends LinearOpMode {

    static FieldTracker tracker;
    BNO055IMU imu;

    public void runOpMode() {
        tracker = new FieldTracker(hardwareMap, telemetry);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        waitForStart();

        double imuAngle, trackerAngle;

        while (opModeIsActive() && !isStopRequested()) {

            imuAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            TargetInfo trackerInfo = tracker.getTargetInfo();

            telemetry.addData("IMU Rotation", imuAngle);

            if (trackerInfo != null) {
                trackerAngle = trackerInfo.zRotation;
                telemetry.addData("Tracker Z Rotation", trackerAngle);
                telemetry.addData("Difference", imuAngle - trackerAngle);
            }

            telemetry.addData("\n\nTracker Info", tracker.getTargetInfo());
            telemetry.update();
        }
    }

}
