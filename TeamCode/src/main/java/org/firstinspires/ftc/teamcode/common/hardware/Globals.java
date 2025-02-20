package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.gamespecific.Side;

public class Globals {
    public static Side COLOR = Side.RED;
    // Match constants
    public static Side SIDE = Side.LEFT;
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = true;
    public static boolean USING_DASHBOARD = false;

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;
    public static boolean threeSpec = false;
    public static boolean IS_TRANSFER = false;
    public static boolean RESET_ENCODER = true;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void stopScoring() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void retract() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void startTransferring() {
        IS_TRANSFER = true;
    }

    public static void stopTransferring() {
        IS_TRANSFER = false;
    }
}
