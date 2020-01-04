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

    public boolean withinRange (Position otherPosition, double xMaxError, double yMaxError, double headingMaxError) {
        double xError = getXDifference(otherPosition);
        double yError = getYDifference(otherPosition);
        double headingError = getHeadingDifference(otherPosition);
        return xError < xMaxError && yError < yMaxError && headingError < headingMaxError;
    }

    //returns abs value
    public double getXDifference (Position otherPosition) {
        return Math.abs(this.x - otherPosition.x);
    }

    //returns abs value
    public double getYDifference (Position otherPosition) {
        return Math.abs(this.y - otherPosition.y);
    }

    //returns abs value
    public double getHeadingDifference (Position otherPosition) {
        return this.heading.getDifference(otherPosition.heading);
    }

    //returns unit vector FROM this position TO target position
    public Vector2d getDirectionTo (Position targetPosition) {
        return new Vector2d(getXDifference(targetPosition), getYDifference(targetPosition)).getUnitVector();
    }

    //returns Direction FROM this position TO target position
    public Angle.Direction getRotationDirectionTo (Position targetPosition) {
        return this.heading.directionTo(targetPosition.heading);
    }


    //completely resets robot position
    public void reset () {
        x = 0;
        y = 0;
        heading = new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING);
    }
}
