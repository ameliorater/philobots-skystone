package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Vector", group = "TeleOp")
public class TestVector extends OpMode {

    public void init () {}

    public void loop () {
        Vector2d right = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        Vector2d rightNotFlipped = new Vector2d(gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Right", right);
        telemetry.addData("Right Not Flipped", rightNotFlipped);
        telemetry.addData("Right Angle", right.getAngle());
        telemetry.addData("Right Not Flipped Angle", rightNotFlipped.getAngle());
        telemetry.update();
    }
}
