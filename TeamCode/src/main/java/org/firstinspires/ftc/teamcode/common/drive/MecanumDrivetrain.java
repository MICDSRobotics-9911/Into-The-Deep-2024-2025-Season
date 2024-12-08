package org.firstinspires.ftc.teamcode.common.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.Locale;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class MecanumDrivetrain extends WSubsystem implements Drivetrain {
    private final RobotHardware robot = RobotHardware.getInstance();

    public double[] wheelPowers;

    public MecanumDrivetrain() {
        wheelPowers = new double[4];
    }
    @Override
    public void set(Pose pose) {
        set(pose, 0);
    }

    public double scaleInput(double input) {
        return (0.5 * Math.tan(1.12 * input));
    }

    public void set(double forwardSpeed,double strafeSpeed,
                    double turnSpeed, double gyroAngle) {
        if (!Globals.IS_AUTO) {
            /*forwardSpeed = forwardSpeed * Math.cos(-gyroAngle) - strafeSpeed * Math.sin(-gyroAngle);
            strafeSpeed = forwardSpeed * Math.sin(-gyroAngle) - strafeSpeed * Math.sin(-gyroAngle);*/
            strafeSpeed = scaleInput(strafeSpeed);
            forwardSpeed = scaleInput(forwardSpeed);
            //turnSpeed = scaleInput(turnSpeed);
            strafeSpeed *= 1.1;
        }


        Vector2D input = new Vector2D(strafeSpeed, forwardSpeed).rotate(-gyroAngle);

        strafeSpeed = Range.clip(input.x, -1, 1);
        forwardSpeed = Range.clip(input.y, -1, 1);
        turnSpeed = Range.clip(turnSpeed, -1, 1);

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[0] = forwardSpeed + strafeSpeed + turnSpeed;
        wheelSpeeds[1] = forwardSpeed - strafeSpeed - turnSpeed;
        wheelSpeeds[2] = forwardSpeed - strafeSpeed + turnSpeed;
        wheelSpeeds[3] = forwardSpeed + strafeSpeed - turnSpeed;


        if (Globals.IS_AUTO) {
            // feedforward and voltage comp
            /*double correction = 12 / robot.getVoltage();
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = Math.abs(wheelSpeeds[i]) < 0.01 ?
                        wheelSpeeds[i] * correction :
                        (wheelSpeeds[i] + Math.signum(wheelSpeeds[i]) * 0.085) * correction;
            }*/
        }

        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));
        if (max > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++)
                wheelSpeeds[i] /= max;
        }

        for (int i = 0; i < wheelSpeeds.length; i++)
            wheelPowers[i] = wheelSpeeds[i];
    }

    public void set(Pose pose, double angle) {
        set(pose.x, pose.y, pose.heading, angle);
    }

    public void setPower() {
        robot.frontLeft.setPower(wheelPowers[0]);
        robot.frontRight.setPower(wheelPowers[1]);
        robot.backLeft.setPower(wheelPowers[2]);
        robot.backRight.setPower(wheelPowers[3]);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void read() {
        robot.odo.update();
    }

    @Override
    public void write() {
        setPower();
    }

    @Override
    public void reset() {
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = 0;
        }
    }

    @NonNull
    @Override
    public String toString() {
        Pose2D robotPose = robot.getPose();
        return String.format(
                Locale.US,
                "FLPower: %.2f\nFRPower: %.2f\nBLPower: %.2f\nBRPower: %.2f\n" +
                        "x: %.3f\ny: %.3f\nheading: %.3f\n",
                wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3],
                robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH),
                robotPose.getHeading(AngleUnit.RADIANS)
        );
    }
}
