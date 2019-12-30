package org.firstinspires.ftc.teamcode;

public class Position {
    //stores an absolute position on field
    double x, y;
    Angle heading;

    public Position (double x, double y, Angle heading) {
        this.x = x;
        this.y = y;
        this.heading = heading; //todo: may want to force type
    }

    public void set (double x, double y, Angle heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void increment (double deltaX, double deltaY, double deltaHeading) {
        this.x += deltaX;
        this.y += deltaY;
        //todo: make this built into angle class/ check if it will work for all types/cases
        this.heading = new Angle(heading.getAngle() + deltaHeading, heading.getType());
    }

    public void incrementX (double deltaX) {
        this.x += deltaX;
    }

    public void incrementY (double deltaY) {
        this.y += deltaY;
    }

    public void incrementHeading (double deltaHeading) {
        //todo: make this built into angle class/ check if it will work for all types/cases
        //this.heading = new Angle(heading.getAngle() + deltaHeading, heading.getType());
        heading = heading.rotateBy(deltaHeading, Angle.Direction.CLOCKWISE);
    }

    //completely resets robot position
    public void reset () {
        x = 0;
        y = 0;
        heading = new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING);
    }
}
