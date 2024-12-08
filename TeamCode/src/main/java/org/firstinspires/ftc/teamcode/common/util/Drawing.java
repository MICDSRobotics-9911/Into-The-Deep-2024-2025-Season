package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2D pose) {
        final double ROBOT_RADIUS = 9;
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        c.setStroke("green");
        c.strokeCircle(x, y, ROBOT_RADIUS);
        c.strokeLine(x, y, x + Math.cos(heading) * ROBOT_RADIUS,
                y + Math.sin(heading) * ROBOT_RADIUS);
    }
}