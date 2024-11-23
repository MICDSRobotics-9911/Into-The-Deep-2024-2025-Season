package org.firstinspires.ftc.teamcode.common.commandbased.drivecommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.util.Pose;


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

        Pose robotPose = (Pose) robot.values.get(Sensors.SensorType.PINPOINT);

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose robotPose = (Pose) robot.values.get(Sensors.SensorType.PINPOINT);
        Pose delta = targetPose.subtract(robotPose);
        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
    }

    public Pose getPower(Pose robotPose) {
        while (targetPose.heading - robotPose.heading > Math.PI)
            targetPose.heading -= 2 * Math.PI;
        while (targetPose.heading - robotPose.heading < Math.PI)
            targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = yPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -1, 1);
        x_rotated = Range.clip(x_rotated, -1, 1);
        y_rotated = Range.clip(y_rotated, -1, 1);

        return new Pose(x_rotated, y_rotated, hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }

}
