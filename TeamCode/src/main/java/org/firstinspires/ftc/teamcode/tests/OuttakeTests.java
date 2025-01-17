package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.LinkedServos;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@TeleOp(name="OuttakeTests")
public class OuttakeTests extends LinearOpMode {
    private CachingServo outtakeClaw;
    private CachingServo outtakeArmLeft;
    private CachingServo outtakeArmRight;
    private CachingDcMotorEx extensionRight;
    private CachingDcMotorEx extensionLeft;
    private LinkedServos outtakeArm;
    public static double leftArmPos = 0.5;
    public static double rightArmPos = 0.5;
    public static double rightClawPos = 0.5;
    public static double leftClawPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        extensionRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionRight"));
        extensionLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionLeft"));
        outtakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmLeft"));
        outtakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmRight"));
        outtakeClaw = new CachingServo(hardwareMap.get(Servo.class, "outtakeClaw"));
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                outtakeClaw.setPosition(0.9);
            }
            if (gamepad1.dpad_down) {
                outtakeClaw.setPosition(0.3);
            }
            telemetry.addData("claw_position: ", outtakeClaw.getPosition());
            if (gamepad1.a) {
                outtakeArmLeft.setPosition(1);
                outtakeArmRight.setPosition(1);
            }
            if (gamepad1.b) {
                outtakeArmLeft.setPosition(0);
                outtakeArmRight.setPosition(0);
            }
            if (gamepad2.dpad_up) {
                extensionLeft.setPower(1);
                extensionRight.setPower(1);
            } else if (gamepad2.dpad_down) {
                extensionLeft.setPower(-1);
                extensionRight.setPower(-1);
            } else {
                extensionLeft.setPower(0);
                extensionRight.setPower(0);
            }
            telemetry.addData("left_arm_position: ", outtakeArmLeft.getPosition());
            telemetry.update();
        }
    }
}
