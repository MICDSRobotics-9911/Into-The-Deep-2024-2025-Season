package org.firstinspires.ftc.teamcode.common.util;

import java.util.ArrayList;
import java.util.Arrays;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class LinkedMotors {
    private ArrayList<CachingDcMotorEx> motors;
    public LinkedMotors(CachingDcMotorEx... motors) {
        this.motors = new ArrayList<CachingDcMotorEx>();
        this.motors.addAll(Arrays.asList(motors));
    }

    public void setPower(double power) {
        for (CachingDcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    public int getCurrentPosition() {
        double sum = 0;
        for (CachingDcMotorEx motor : motors)
            sum += motor.getCurrentPosition();
        return (int) Math.round(sum / motors.size());
    }

    public ArrayList<CachingDcMotorEx> getMotors() {
        return motors;
    }
}
