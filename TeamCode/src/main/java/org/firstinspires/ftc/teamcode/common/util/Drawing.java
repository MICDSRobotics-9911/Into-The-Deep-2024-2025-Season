package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.canvas.Canvas;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose pose) {
        final double ROBOT_RADIUS = 9;

        c.setStroke("green");
        c.strokeCircle(pose.x, pose.y, ROBOT_RADIUS);
        c.strokeLine(pose.x, pose.y, pose.x + Math.cos(pose.heading) * ROBOT_RADIUS,
                pose.y + Math.sin(pose.heading) * ROBOT_RADIUS);
    }
}