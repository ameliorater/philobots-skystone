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



//    public void setAbsPosition (double absXPos, double absYPos) {
//        this.absXPos = absXPos;
//        this.absYPos = absYPos;
//    }

}