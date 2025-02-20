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
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Pose;


@Config
public class PositionCommand extends CommandBase {
    private Drivetrain drivetrain;
    public Pose targetPose;
    private double maxSpeed = 0.5;


    public static double xP = 0.7;
    public static double xD = 0.01;

    public static double yP = 0.7;
    public static double yD = 0.01;

    public static double hP = 3.5;
    public static double hD = 0.1;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0.0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public double ALLOWED_TRANSLATIONAL_ERROR = 5;
    public double ALLOWED_HEADING_ERROR = 5;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double STABLE_MS = 100;
    public static double DEAD_MS = 5000;


    public PositionCommand(Pose targetPose) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;
        if (Globals.threeSpec) {
            maxSpeed = 0.7;
        } else if (maxSpeed == 0.5) {
            xP = 0.5;
            xD = 0;
            yP = 0.5;
            yD = 0;
            hP = 3.5;
            hD = 0.1;
        } else if (maxSpeed == 0.3) {
            xP = 0.1;
            xD = 0.01;
            yP = 0.1;
            yD = 0.01;
            hP = 4;
            hD = 0.1;
        }
        xController.reset();
        yController.reset();
        hController.reset();
    }

    public PositionCommand(Pose targetPose, double speed) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;
        if (maxSpeed == 0.7) {
            yP = 0.065;
            yD = 0.01;
        } else if (maxSpeed == 0.5) {
            xP = 0.5;
            xD = 0;
            yP = 0.5;
            yD = 0;
            hP = 3.5;
            hD = 0.1;
        } else if (maxSpeed == 0.3) {
            xP = 0.1;
            xD = 0.01;
            yP = 0.1;
            yD = 0.01;
            hP = 4;
            hD = 0.1;
        }

        maxSpeed = speed;
        xController.reset();
        yController.reset();
        hController.reset();
        xController.setPIDF(xP, 0, xD, 0);
        yController.setPIDF(yP, 0, yD, 0);
        hController.setPIDF(hP, 0, hD, 0);
    }

    public PositionCommand(Pose targetPose, double speed, double translationalTolerance, double headingTolerance) {
        this.drivetrain = robot.drivetrain;
        this.targetPose = targetPose;
        maxSpeed = speed;
        if (maxSpeed == 0.7) {
            yP = 0.065;
            yD = 0.01;
        } else if (maxSpeed == 0.5) {
            xP = 0.4;
            xD = 0;
            yP = 0.4;
            yD = 0;
            hP = 3.5;
            hD = 0.1;
        } else if (maxSpeed == 0.3) {
            xP = 0.8;
            xD = 0.01;
            yP = 0.8;
            yD = 0.01;
            hP = 4;
            hD = 0.1;
        }
        xController.reset();
        yController.reset();
        hController.reset();
        ALLOWED_TRANSLATIONAL_ERROR = translationalTolerance;
        ALLOWED_HEADING_ERROR = headingTolerance;
    }

    @Override
    public void execute() {
        xController.setPIDF(xP, 0, xD, 0);
        yController.setPIDF(yP, 0, yD, 0);
        hController.setPIDF(hP, 0, hD, 0);
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose2D robotPose = robot.odo.getPosition();
        //System.out.println(targetPose);

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        if (!atPoint()) {
            stable.reset();
        }

        System.out.println(robot.getPose());

        return /*timer.milliseconds() > DEAD_MS ||*/ stable.milliseconds() > STABLE_MS;
    }

    double xThreshold = ALLOWED_TRANSLATIONAL_ERROR;
    double yThreshold = ALLOWED_TRANSLATIONAL_ERROR;
    double turnThreshold = Math.toRadians(ALLOWED_HEADING_ERROR);

    public boolean atPoint() {
        return Math.abs(targetPose.x - robot.getPose().getX(DistanceUnit.INCH)) < xThreshold &&
                Math.abs(targetPose.y - robot.getPose().getY(DistanceUnit.INCH)) < yThreshold &&
                Math.abs(robot.getPose().getHeading(AngleUnit.RADIANS) - targetPose.heading) <
                        turnThreshold;
    }


    public Pose getPower(Pose2D robotPose) {
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        double x = robotPose.getX(DistanceUnit.INCH);
        double y = robotPose.getY(DistanceUnit.INCH);
        while (targetPose.heading - heading > Math.PI)
            targetPose.heading -= 2 * Math.PI;
        while (targetPose.heading - heading < -Math.PI)
            targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(x, targetPose.x);
        double yPower = -yController.calculate(y, targetPose.y);
        double hPower = -hController.calculate(heading, targetPose.heading);


        double x_rotated = xPower * Math.cos(heading) - yPower * Math.sin(heading);
        double y_rotated = xPower * Math.sin(heading) + yPower * Math.cos(heading);
        if (Globals.threeSpec) {
            y_rotated = yPower * Math.sin(heading) + yPower * Math.cos(heading);
        }

        double maxPower = Math.max(1.0, Math.max(Math.abs(x_rotated), Math.max(Math.abs(y_rotated), Math.abs(hPower))));

        if (Globals.threeSpec) {
            hPower = Range.clip(hPower, -maxSpeed, maxSpeed);
            x_rotated = Range.clip(x_rotated, -maxSpeed, maxSpeed);
            y_rotated = Range.clip(y_rotated, -maxSpeed, maxSpeed);
        } else {
            x_rotated = (x_rotated / maxPower) * maxSpeed;
            y_rotated = (y_rotated / maxPower) * maxSpeed;
            hPower = (hPower / maxPower) * maxSpeed;
        }


        // SQUID STUFF
        /*hPower = MathUtils.signSqrt(hPower);
        x_rotated = MathUtils.signSqrt(x_rotated);
        y_rotated = MathUtils.signSqrt(y_rotated);*/

        return new Pose(x_rotated, y_rotated, hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }

}
