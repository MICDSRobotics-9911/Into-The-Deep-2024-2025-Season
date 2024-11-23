package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.LinkedServos;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@TeleOp(name="OuttakeTests")
public class OuttakeTests extends LinearOpMode {
    private CachingServo outtakeClaw;
    private CachingServo outtakeArmLeft;
    private CachingServo outtakeArmRight;
    private LinkedServos outtakeArm;
    public static double leftArmPos = 0;
    public static double rightArmPos = 0;
    public static double clawPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        outtakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmLeft"));
        outtakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmRight"));
        outtakeClaw = new CachingServo(hardwareMap.get(Servo.class, "outtakeClaw"));
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                outtakeClaw.setPosition(clawPos);
            }
            if (gamepad1.dpad_down) {
                outtakeClaw.setPosition(0);
            }
            telemetry.addData("claw_position: ", outtakeClaw.getPosition());
            if (gamepad1.a) {
                outtakeArmLeft.setPosition(leftArmPos);
            }
            if (gamepad1.b) {
                outtakeArmLeft.setPosition(0);
            }
            telemetry.addData("left_arm_position: ", outtakeArmLeft.getPosition());
            telemetry.update();
        }
    }
}
