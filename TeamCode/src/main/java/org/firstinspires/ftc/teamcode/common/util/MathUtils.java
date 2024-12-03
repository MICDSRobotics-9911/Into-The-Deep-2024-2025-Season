package org.firstinspires.ftc.teamcode.common.util;

public class MathUtils {
    public static double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private static double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

    public static double signSqrt(double num) {
        int sign = 1;
        if (num < 0)
            sign = -1;
        return sign * Math.sqrt(num);
    }
}
