package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="FieldTrackingTestv2", group ="Auto")
public class FieldTrackingTester extends LinearOpMode {

    static FieldTracker tracker;

    public void runOpMode() {
        tracker = new FieldTracker(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            tracker.logInfo();

        }
    }

}
