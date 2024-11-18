package org.firstinspires.ftc.teamcode.common.util;

import java.util.ArrayList;
import java.util.Arrays;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class LinkedMotors {
    private ArrayList<CachingDcMotorEx> motors;
    public LinkedMotors(CachingDcMotorEx... motors) {
        this.motors.addAll(Arrays.asList(motors));
    }

    public void setPower(double power) {
        for (CachingDcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public double getCurrentPosition() {
        double sum = 0;
        for (CachingDcMotorEx motor : motors)
            sum += motor.getCurrentPosition();
        return sum / motors.size();
    }

    public ArrayList<CachingDcMotorEx> getMotors() {
        return motors;
    }
}
