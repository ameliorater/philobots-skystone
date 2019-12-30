package org.firstinspires.ftc.teamcode;

public class Measurement {

    public final static int CM_UNIT = 0, MM_UNIT = 1, INCH_UNIT = 2, FEET_UNIT = 3;

    private final static double[][] conversionFactors = {
            {1,         0.1,        2.54,       30.48   },
            {10,        1,          25.4,       304.8   },
            {1 / 2.54,  1 / 25.4,   1,          12      },
            {1 / 30.48, 1 / 304.8,  1 / 12.0,   1       }
    };

    public static double convert(double initialValue, int fromUnit, int toUnit) {
        return initialValue * conversionFactors[fromUnit][toUnit];
    }

}