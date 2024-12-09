package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class Drawing {
    private Drawing() {}


    public static TelemetryPacket drawRobot(Pose2D pose) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Canvas c = packet.fieldOverlay();
        final double ROBOT_RADIUS = 9;
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        c.setStroke("green");
        c.strokeCircle(x, y, ROBOT_RADIUS);
        c.strokeLine(x, y, x + Math.cos(heading) * ROBOT_RADIUS,
                y + Math.sin(heading) * ROBOT_RADIUS);
        return packet;
    }
}