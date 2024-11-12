package org.firstinspires.ftc.teamcode.common.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.Locale;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.core.dependency.Dependency;

public class Drivetrain extends WSubsystem {
    // FL, FR, BL, BR
    public CachingDcMotorEx[] motors;
    public double[] wheelPowers;
    private double[] wheelPos;

    public Drivetrain(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, IMU imu) {
        motors[0] = new CachingDcMotorEx(frontLeft);
        motors[1] = new CachingDcMotorEx(frontRight);
        motors[2] = new CachingDcMotorEx(backLeft);
        motors[3] = new CachingDcMotorEx(backRight);

        wheelPowers = new double[4];
        wheelPos = new double[4];
    }

    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = power * cos / max + turn;
        double rightFront = power * sin / max - turn;
        double leftBack = power * sin / max + turn;
        double rightBack = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftBack /= power + Math.abs(turn);
            rightBack /= power + Math.abs(turn);
        }
        wheelPowers[0] = leftFront;
        wheelPowers[1] = rightFront;
        wheelPowers[2] = leftBack;
        wheelPowers[3] = rightBack;
    }

    public void setPower(double power) {
        for (int i = 0; i < 4; i++)
            motors[i].setPower(power);
    }

    public void setPower() {
        for (int i = 0; i < 4; i++)
            motors[i].setPower(wheelPowers[i]);
    }

    public void getPosition() {
        /*for (int i = 0; i < 4; i++)
            wheelPos[i] = motors[i].getCurrentPosition();*/
    }


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return null;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {

    }

    @Override
    public void periodic() {
        if (Globals.IS_AUTO) {
            read();
        }
        write();
    }

    @Override
    public void read() {
        getPosition();
    }

    @Override
    public void write() {
        setPower();
    }

    @Override
    public void reset() {
        setPower(0);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(
                Locale.US,
                "FLPower: %.2f\nFRPower: %.2f\nBLPower: %.2f\nBRPower: %.2f\n",
                wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]
        );
    }
}
