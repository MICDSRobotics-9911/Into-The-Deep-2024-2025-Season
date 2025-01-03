package org.firstinspires.ftc.teamcode.common.commandbased.drivecommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.Pose;

import java.util.Objects;


@Config
public class PositionCommand extends CommandBase {
    private Drivetrain drivetrain;
    public Pose targetPose;

    public static double xP = 2;
    public static double xD = 0.001;

    public static double yP = 2;
    public static double yD = 0.001;

    public static double hP = 4;
    public static double hD = 0.01;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0.0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = 0.02;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double STABLE_MS = 100;
    public static double DEAD_MS = 2500;

    public PositionCommand(Pose targetPose) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;

        xController.reset();
        yController.reset();
        hController.reset();
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose2D robotPose = robot.odo.getPosition();

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        return atPoint();
    }

    public static double xThreshold = 3;
    public static double yThreshold = 3;
    public static double turnThreshold = 1;
    public boolean atPoint() {
        return Math.abs(targetPose.x - robot.getPose().getX(DistanceUnit.INCH)) < xThreshold &&
                Math.abs(targetPose.y - robot.getPose().getY(DistanceUnit.INCH)) < yThreshold &&
                Math.abs(robot.getPose().getHeading(AngleUnit.RADIANS) - targetPose.heading) <
                        Math.toRadians(turnThreshold);
    }


    public Pose getPower(Pose2D robotPose) {
        double wrappedHeading = targetPose.heading;
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        double x = robotPose.getX(DistanceUnit.INCH);
        double y = robotPose.getY(DistanceUnit.INCH);
        while (wrappedHeading - heading > Math.PI)
            wrappedHeading -= 2 * Math.PI;
        while (wrappedHeading - heading < Math.PI)
            wrappedHeading += 2 * Math.PI;

        double xPower = xController.calculate(x, targetPose.x);
        double yPower = yController.calculate(y, targetPose.y);
        double hPower = -hController.calculate(wrappedHeading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-heading) - yPower * Math.sin(-heading);
        double y_rotated = yPower * Math.sin(-heading) + yPower * Math.cos(-heading);
        x_rotated = xPower;
        y_rotated = yPower;

        hPower = Range.clip(hPower, -1, 1);
        x_rotated = Range.clip(x_rotated, -1, 1);
        y_rotated = Range.clip(y_rotated, -1, 1);


        // SQUID STUFF
        /*hPower = MathUtils.signSqrt(hPower);
        x_rotated = MathUtils.signSqrt(x_rotated);
        y_rotated = MathUtils.signSqrt(y_rotated);*/

        return new Pose(x_rotated, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }

}
