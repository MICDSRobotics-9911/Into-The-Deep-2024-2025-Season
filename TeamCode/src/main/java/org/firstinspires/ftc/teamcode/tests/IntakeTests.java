package org.firstinspires.ftc.teamcode.tests;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
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
    private CachingServo intakeCoaxial;
    public static double turretPos = 0.46;
    public static double coaxialPos = 0.4;
    public static double extensionPos = 0.32;
    public static double retractionPos = 0;
    public static double armPos = 0.8;
    public static double intakeClawPos = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "intakeArmLeft"));
        intakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "intakeArmRight"));
        intakeArmRight.setDirection(REVERSE);
        intakeCoaxial = new CachingServo(hardwareMap.get(Servo.class, "intakeCoaxial"));
        intakeCoaxial.setDirection(FORWARD);
        intakeClaw = new CachingServo(hardwareMap.get(Servo.class, "intakeClaw"));
        turretClaw = new CachingServo(hardwareMap.get(Servo.class, "turretClaw"));
        linkageServoLeft = new CachingServo(hardwareMap.get(Servo.class, "linkageServoLeft"));
        linkageServoLeft.setDirection(REVERSE);
        linkageServoRight = new CachingServo(hardwareMap.get(Servo.class, "linkageServoRight"));
        waitForStart();
        while (opModeIsActive()) {
            // TODO: DO IT LATER BUT YOU HAVE TO COMPLETELY RESET THESE BOUNDS ON THE SERVO
            if (gamepad1.dpad_right) {
                linkageServoLeft.setPosition(retractionPos);
                linkageServoRight.setPosition(retractionPos);
            }
            if (gamepad1.dpad_left) {
                linkageServoLeft.setPosition(extensionPos);
                linkageServoRight.setPosition(extensionPos);
            }
            if (gamepad1.dpad_up) {
                intakeClaw.setPosition(0.9);
            }
            if (gamepad1.dpad_down) {
                intakeClaw.setPosition(intakeClawPos);
            }
            telemetry.addData("claw_position: ", intakeClaw.getPosition());
            if (gamepad1.a) {
                intakeArmLeft.setPosition(1);
                intakeArmRight.setPosition(1);
            }
            if (gamepad1.b) {
                intakeArmLeft.setPosition(armPos);
                intakeArmRight.setPosition(armPos);
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