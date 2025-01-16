package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.LinkedServos;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@TeleOp(name="IntakeTests")
public class IntakeTests extends LinearOpMode {
    private CachingServo intakeClaw;
    private CachingServo turretClaw;
    private CachingServo intakeArmLeft;
    private CachingServo intakeArmRight;
    private CachingServo linkageServoLeft;
    private CachingServo linkageServoRight;
    public static double leftArmPos = 0.5;
    public static double rightArmPos = 0.5;
    public static double rightClawPos = 0.5;
    public static double leftClawPos = 0.5;
    public static double linkagePos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "intakeArmLeft"));
        intakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "intakeArmRight"));
        intakeClaw = new CachingServo(hardwareMap.get(Servo.class, "intakeClaw"));
        turretClaw = new CachingServo(hardwareMap.get(Servo.class, "turretClaw"));
        linkageServoLeft = new CachingServo(hardwareMap.get(Servo.class, "linkageServoLeft"));
        linkageServoRight = new CachingServo(hardwareMap.get(Servo.class, "linkageServoRight"));
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                linkageServoLeft.setPosition(linkagePos);
                linkageServoRight.setPosition(linkagePos);
            }
            if (gamepad1.dpad_up) {
                intakeClaw.setPosition(rightClawPos);
            }
            if (gamepad1.dpad_down) {
                intakeClaw.setPosition(leftClawPos);
            }
            telemetry.addData("claw_position: ", intakeClaw.getPosition());
            if (gamepad1.a) {
                intakeArmLeft.setPosition(leftArmPos);
                intakeArmRight.setPosition(rightArmPos);
            }
            if (gamepad1.b) {
                intakeArmLeft.setPosition(0);
                intakeArmRight.setPosition(0);
            }
            telemetry.addData("left_arm_position: ", intakeArmLeft.getPosition());
            telemetry.update();
        }
    }
}