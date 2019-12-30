package org.firstinspires.ftc.teamcode;

public class Measurement {

    static int CM_INDEX = 0, MM_INDEX = 1, INCH_INDEX = 2, FEET_INDEX = 3;

    static double[][] conversionFactors = {
            {1,         0.1,        2.54,       30.48   },
            {10,        1,          25.4,       304.8   },
            {1 / 2.54,  1 / 25.4,   1,          12      },
            {1 / 30.48, 1 / 304.8,  1 / 12.0,   1       }
    };

    static double convert(double initialValue, int fromUnit, int toUnit) {
        return initialValue * conversionFactors[fromUnit][toUnit];
    }

}