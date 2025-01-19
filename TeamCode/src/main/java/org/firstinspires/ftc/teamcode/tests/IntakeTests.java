package org.firstinspires.ftc.teamcode.tests;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

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
    private CachingServo intakeCoaxial;
    public static double turretPos = 0.5;
    public static double coaxialPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "intakeArmLeft"));
        intakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "intakeArmRight"));
        intakeArmRight.setDirection(REVERSE);
        intakeCoaxial = new CachingServo(hardwareMap.get(Servo.class, "intakeCoaxial"));
        intakeCoaxial.setDirection(REVERSE);
        intakeClaw = new CachingServo(hardwareMap.get(Servo.class, "intakeClaw"));
        turretClaw = new CachingServo(hardwareMap.get(Servo.class, "turretClaw"));
        linkageServoLeft = new CachingServo(hardwareMap.get(Servo.class, "linkageServoLeft"));
        linkageServoRight = new CachingServo(hardwareMap.get(Servo.class, "linkageServoRight"));
        waitForStart();
        while (opModeIsActive()) {
            // TODO: DO IT LATER BUT YOU HAVE TO COMPLETELY RESET THESE BOUNDS ON THE SERVO
            if (gamepad1.dpad_right) {
                linkageServoLeft.setPosition(0);
                linkageServoRight.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                linkageServoLeft.setPosition(0.5);
                linkageServoRight.setPosition(0.5);
            }
            if (gamepad1.dpad_up) {
                intakeClaw.setPosition(0.9);
            }
            if (gamepad1.dpad_down) {
                intakeClaw.setPosition(0.75);
            }
            telemetry.addData("claw_position: ", intakeClaw.getPosition());
            if (gamepad1.a) {
                intakeArmLeft.setPosition(1);
                intakeArmRight.setPosition(1);
            }
            if (gamepad1.b) {
                intakeArmLeft.setPosition(0.9);
                intakeArmRight.setPosition(0.9);
            }
            if (gamepad1.x) {
                turretClaw.setPosition(turretPos);
            }
            if (gamepad1.left_stick_button) {
                intakeCoaxial.setPosition(1);
            }
            if (gamepad1.right_stick_button) {
                intakeCoaxial.setPosition(coaxialPos);
            }
            telemetry.addData("left_arm_position: ", intakeArmLeft.getPosition());
            telemetry.update();
        }
    }
}