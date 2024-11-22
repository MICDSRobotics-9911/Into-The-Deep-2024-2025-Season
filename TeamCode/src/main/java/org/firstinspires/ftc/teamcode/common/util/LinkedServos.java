package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class LinkedServos {
    private ArrayList<CachingServo> servos;

    public LinkedServos(CachingServo... servos) {
        this.servos = new ArrayList<CachingServo>();
        this.servos.addAll(Arrays.asList(servos));
    }

    public void setPosition(double position) {
        for (CachingServo servo : servos)
            servo.setPosition(position);
    }

    public double getPosition() {
        double sum = 0;
        for (CachingServo servo : servos) {
            sum += servo.getPosition();
        }
        return sum / servos.size();
    }

    public ArrayList<CachingServo> getServos() {
        return servos;
    }

}
