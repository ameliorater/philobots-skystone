package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

public class SCARAController {
    double arm1Length, arm2Length;

    double theta1; // angle of arm1 relative to the frame
    double theta2; // angle of arm2 relative to arm1

    ClawPosition clawInsideRobot;
    ClawPosition clawUnderBridge;
    ClawPosition clawOutsideRobot;
    ClawPosition clawAtDelivery;

    ClawPosition lastPosition;

    Telemetry telemetry;

    double servo1TicksPerRadian, servo2TicksPerRadian;

    final static double ARM_SPEED = 250; // mm per second

    final static double SIDE_TO_SIDE_RANGE = 8 * 25.4;
    final static double MIDLINE = -76;
    final static double CALIBRATION_Y_DISTANCE = 200;
    final static double DELIVER_Y_DISTANCE = 5.25 * 25.4;


    public SCARAController(double arm1Length, double arm2Length, Telemetry telemetry) {
        this.arm1Length = arm1Length;
        this.arm2Length = arm2Length;
        this.telemetry = telemetry;

        ArmAngles armAngles = new ArmAngles(0,0);
        clawInsideRobot = new ClawPosition(MIDLINE, -CALIBRATION_Y_DISTANCE, .119, .883);
        clawUnderBridge = new ClawPosition(MIDLINE, 0, .202, .411);
//        clawOutsideRobot = new ClawPosition(MIDLINE, OUTSIDE_DISTANCE, .801, .346);
        clawOutsideRobot = new ClawPosition(MIDLINE, CALIBRATION_Y_DISTANCE, .690, .375);
        clawAtDelivery = new ClawPosition(MIDLINE, DELIVER_Y_DISTANCE, .520, .357);

//        servo1TicksPerRadian = ((clawOutsideRobot.servoPositions.servo1 - clawInsideRobot.servoPositions.servo1) / -getAngleDifferenceCounterClockwise(clawOutsideRobot.armAngles.angle1, clawInsideRobot.armAngles.angle1));
//        servo2TicksPerRadian = (clawInsideRobot.servoPositions.servo2 - clawUnderBridge.servoPositions.servo2)
////                / getAngleDifferenceClockwise(getAngleDifferenceClockwise(clawInsideRobot.theta1, clawInsideRobot.theta2), getAngleDifferenceClockwise(clawUnderBridge.theta1, clawUnderBridge.theta2)));
//                / getAngleDifferenceCounterClockwise(clawUnderBridge.armAngles.angle1 + clawUnderBridge.armAngles.angle2, clawInsideRobot.armAngles.angle1 + clawInsideRobot.armAngles.angle2);

        servo1TicksPerRadian = getServo1TicksPerRadian(clawInsideRobot, clawOutsideRobot);
        servo2TicksPerRadian = getServo2TicksPerRadian(clawInsideRobot, clawUnderBridge);
//        servo1TicksPerRadian = getServo1TicksPerRadian(clawInsideRobot, clawAtDelivery);
//        servo2TicksPerRadian = getServo2TicksPerRadian(clawInsideRobot, clawAtDelivery);

        lastPosition = clawInsideRobot;
    }


    public double getServo1TicksPerRadian(ClawPosition p1, ClawPosition p2) {
        return ((p2.servoPositions.servo1 - p1.servoPositions.servo1) / -getAngleDifferenceCounterClockwise(p2.armAngles.angle1, p1.armAngles.angle1));
    }

    public double getServo2TicksPerRadian(ClawPosition p1, ClawPosition p2) {
        return (p1.servoPositions.servo2 - p2.servoPositions.servo2) / getAngleDifferenceCounterClockwise(p2.armAngles.angle1 + p2.armAngles.angle2, p1.armAngles.angle1 + p1.armAngles.angle2);

    }

//    public boolean calculateAngles(double x, double y, boolean inverted, ArmAngles armAngles) {
//        double cos_theta2 = (x*x + y*y - arm1Length * arm1Length - arm2Length * arm2Length) / (2 * arm1Length * arm2Length);
//
//        // first term can be plus or minus resulting in two different solutions for the same position
//        double newTheta2 = Math.atan2((inverted ? -1 : 1) * Math.sqrt(1 - cos_theta2 * cos_theta2), cos_theta2);
//        double k1 = arm1Length + arm2Length * Math.cos(newTheta2);
//        double k2 = arm2Length * Math.sin(newTheta2);
//        double newTheta1 = Math.atan2(y, x) - Math.atan2(k2, k1);
//
//        if (Double.isNaN(newTheta1) || Double.isNaN(newTheta2) || Double.isInfinite(newTheta1) || Double.isInfinite(newTheta2)) {
//            return false;
//        }
//
//        armAngles.angle1 = newTheta1;
//        armAngles.angle2 = newTheta2;
//        return true;
//    }
//
//    public ArmAngles getClawPositionAngles(ClawPosition clawPosition) {
//        ArmAngles armAngles = new ArmAngles(clawPosition.coordinates, clawPosition.inverted);
//        return armAngles;
//    }

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getAngleDifferenceCounterClockwise(double startAngle, double endAngle) {
        if (startAngle <= endAngle) {
            return endAngle - startAngle;
        } else {
            return endAngle - startAngle + 2 * Math.PI;
        }
    }

    /*
     * Return the angle difference travelling clockwise from startAngle to endAngle
     */
    public double getSignedAngleDifferenceCounterClockwise(double startAngle, double endAngle, double threshold) {
        double difference = endAngle - startAngle;
        if (startAngle > endAngle) {
            difference += 2 * Math.PI;
        }
        if (difference > threshold) {
            difference -= 2 * Math.PI;
        }
        return difference;
    }

    /*
     * Store the parameters for a claw location
     * ClawX and ClawY are the coordinates of the claw.  The origin of the coordinate system is the pivot of the SCARA arm.
     * Positive X extends to the left side of the robot, parallel to the back plane of the robot.  Positive Y extends out
     * of the back of the robot
     * arm1Servo and arm2Servo are the observed servo values when the claw is at the given position.  To calibrate these
     * values, the claw should be moved to those coordinates using the test app.  The values displayed should be used here.
     * theta1 and theta2 are the calculated angles of the arms using the inverse kinematics equations.  theta1 is measured
     * relative to the claw's coordinate system above, with increasing angles moving counter-clockwise.  theta2 is measured
     * relative to the first arm.
     *
     * To calculate the servo position for the first arm, scale the servo value by the change in theta1, using two of these
     * control points to define the line.  To calculate the servo position for the second arm, scale the servo value by the change in
     * (theta2 - theta1), using two (possibly different) control points to define the line.
     */
    public class ClawPosition {
        Coordinates coordinates;
        ArmAngles armAngles;
        ServoPositions servoPositions;
        boolean inverted;

        public ClawPosition(double x, double y, double arm1Servo, double arm2Servo) {
            coordinates = new Coordinates(x, y);
            armAngles = new ArmAngles(coordinates, true);
            servoPositions = new ServoPositions(arm1Servo, arm2Servo);
            theta1 = theta2 = 0;
        }

        public ClawPosition(Coordinates coordinates, ServoPositions servoPositions) {
            this(coordinates.x, coordinates.y, servoPositions.servo1, servoPositions.servo2);
        }

        public ClawPosition(ClawPosition other) {
            this(other.coordinates, other.servoPositions);
            this.inverted = other.inverted;
        }

        public boolean moveBy(double deltaX, double deltaY, double deltaTime) {
            if (!coordinates.moveBy(deltaTime * ARM_SPEED * deltaX, deltaTime * ARM_SPEED * deltaY)) return false;
            armAngles.setAngles(coordinates, true);
//            servoPositions.update(armAngles);
            if (coordinates.y < 0)
                servoPositions.updateFromReference(armAngles, clawInsideRobot, clawUnderBridge);
            else
                servoPositions.updateFromReference(armAngles, clawUnderBridge, clawAtDelivery);

            return true;
        }

        public boolean moveTo(double targetX, double targetY, double deltaTime) {
            double deltaX = targetX - coordinates.x;
            if (Math.abs(deltaX) > deltaTime * ARM_SPEED) deltaX = Math.signum(deltaX) * deltaTime * ARM_SPEED;

            double deltaY = targetY - coordinates.y;
            if (Math.abs(deltaY) > deltaTime * ARM_SPEED) deltaY = Math.signum(deltaY) * deltaTime * ARM_SPEED;

            if (!coordinates.moveBy(deltaX, deltaY)) return false;
            armAngles.setAngles(coordinates, true);
//            servoPositions.update(armAngles);
            if (coordinates.y < 0)
                servoPositions.updateFromReference(armAngles, clawInsideRobot, clawUnderBridge);
            else
                servoPositions.updateFromReference(armAngles, clawUnderBridge, clawAtDelivery);
            return true;
        }

    }

    public class ArmAngles {
        double angle1;
        double angle2;

        public ArmAngles(double angle1, double angle2) {
            this.angle1 = angle1;
            this.angle2 = angle2;
        }

        public ArmAngles(ArmAngles other) {
            this(other.angle1, other.angle2);
        }

        public ArmAngles(Coordinates coordinates, boolean inverted) {
            setAngles(coordinates, inverted);
        }

        public boolean setAngles(Coordinates coordinates, boolean inverted) {
            double x = coordinates.x;
            double y = coordinates.y;
            double cos_theta2 = (x*x + y*y - arm1Length * arm1Length - arm2Length * arm2Length) / (2 * arm1Length * arm2Length);

            // first term can be plus or minus resulting in two different solutions for the same position
            double newTheta2 = Math.atan2((inverted ? -1 : 1) * Math.sqrt(1 - cos_theta2 * cos_theta2), cos_theta2);
            double k1 = arm1Length + arm2Length * Math.cos(newTheta2);
            double k2 = arm2Length * Math.sin(newTheta2);
            double newTheta1 = Math.atan2(y, x) - Math.atan2(k2, k1);

            if (Double.isNaN(newTheta1) || Double.isNaN(newTheta2) || Double.isInfinite(newTheta1) || Double.isInfinite(newTheta2)) {
                return false;
            }

            angle1 = newTheta1;
            angle2 = newTheta2;
            return true;
        }

    }

    public class ServoPositions {
        double servo1;
        double servo2;

        public ServoPositions(double servo1, double servo2) {
            this.servo1 = servo1;
            this.servo2 = servo2;
        }

        public ServoPositions(ServoPositions other) {
            this(other.servo1, other.servo2);
        }

        public ServoPositions(ArmAngles armAngles) {
            update(armAngles);
        }

        /*
         * update the servo positions based on the given arm angles.  Return true if the arm angles are within the servos' range
         */
        public boolean update(ArmAngles armAngles) {
            boolean inRange = true;

            // use the position inside of the robot and under the bridge to scale the servo value;
            servo1 = clawInsideRobot.servoPositions.servo1 + getSignedAngleDifferenceCounterClockwise(clawInsideRobot.armAngles.angle1, armAngles.angle1, Math.PI) * servo1TicksPerRadian;
            if (servo1 > 1) {
                servo1 = 1;
                inRange = false;
            }
            if (servo1 < 0) {
                servo1 = 0;
                inRange = false;
            }

            // servo 2's position depends on servo 1's position
            // Theta2 is arm2's angle relative to arm1.  When arm 1's angle changes, servo 2 must rotate to keep the same angle between arm1 and arm2.  This is
            // because arm2's pivot is connected to the servo through a belt which rotates with arm 1.
            // Alternatively define theta2 relative to the frame.
            servo2 = clawInsideRobot.servoPositions.servo2 + getSignedAngleDifferenceCounterClockwise(clawInsideRobot.armAngles.angle1 + clawInsideRobot.armAngles.angle2, armAngles.angle1 + armAngles.angle2, Math.PI * .75)* servo2TicksPerRadian;
            telemetry.addData("start", clawInsideRobot.armAngles.angle1 + clawInsideRobot.armAngles.angle2);
            telemetry.addData("end", armAngles.angle1 + armAngles.angle2);
            if (servo2 > 1) {
                servo2 = 1;
                inRange = false;
            }
            if (servo2 < 0) {
                servo2 = 0;
                inRange = false;
            }

            return inRange;
        }
        public boolean updateFromReference(ArmAngles armAngles, ClawPosition p1, ClawPosition p2) {
            boolean inRange = true;

            // use the position inside of the robot and under the bridge to scale the servo value;
            servo1 = p1.servoPositions.servo1 + getSignedAngleDifferenceCounterClockwise(p1.armAngles.angle1, armAngles.angle1, Math.PI) * getServo1TicksPerRadian(p1, p2);
            if (servo1 > 1) {
                servo1 = 1;
                inRange = false;
            }
            if (servo1 < 0) {
                servo1 = 0;
                inRange = false;
            }

            // servo 2's position depends on servo 1's position
            // Theta2 is arm2's angle relative to arm1.  When arm 1's angle changes, servo 2 must rotate to keep the same angle between arm1 and arm2.  This is
            // because arm2's pivot is connected to the servo through a belt which rotates with arm 1.
            // Alternatively define theta2 relative to the frame.
            servo2 = p1.servoPositions.servo2 + getSignedAngleDifferenceCounterClockwise(p1.armAngles.angle1 + p1.armAngles.angle2, armAngles.angle1 + armAngles.angle2, Math.PI * .75)* getServo2TicksPerRadian(p1, p2);
            if (servo2 > 1) {
                servo2 = 1;
                inRange = false;
            }
            if (servo2 < 0) {
                servo2 = 0;
                inRange = false;
            }

            return inRange;
        }

    }

    public class Coordinates {
        double x, y;

        public Coordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Coordinates(Coordinates other) {
            this(other.x, other.y);
        }

        /*
         * Allowable poositions are within 10mm of the line from clawInsideRobot to clawOutsideRobot or
         * within 10mm of the line from clawOutsideRobot - SIDE_TO_SIDE_RANGE/2 to clawOutsideRobot + SIDE_TO_SIDE_RANGE/2
         */
        public boolean isValid(double newX, double newY) {
            if ((newY >= clawInsideRobot.coordinates.y) && (newY <= DELIVER_Y_DISTANCE) &&
                    (newX >= MIDLINE - 10) && (newX <= MIDLINE + 10)) return true;
            if ((newY <= DELIVER_Y_DISTANCE + 10) && (newY >= DELIVER_Y_DISTANCE - 10) &&
                    (newX >= MIDLINE - SIDE_TO_SIDE_RANGE * 0.5) && (newX <= MIDLINE + SIDE_TO_SIDE_RANGE * 0.5)) return true;
            return false;
        }

        public boolean moveBy(double deltaX, double deltaY) {
            double newX = x + deltaX;
            double newY = y + deltaY;
            if (!isValid(newX, newY)) return false;
            x = newX;
            y = newY;
            return true;
        }
    }
}
