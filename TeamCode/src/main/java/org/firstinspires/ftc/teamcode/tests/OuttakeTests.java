package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.util.LinkedServos;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

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
    public static double slideTarget = 500;
    public static double p = 0.033;
    public static double i = 0;
    public static double d = 0.0003;
    public static double f = 0;
    private PIDFController controller;


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDFController(p, 0, d, 0);
        extensionRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionRight"));
        extensionRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionLeft"));
        extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmLeft"));
        outtakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmRight"));
        outtakeArmRight.setDirection(Servo.Direction.FORWARD);
        outtakeArmLeft.setDirection(Servo.Direction.REVERSE);
        outtakeClaw = new CachingServo(hardwareMap.get(Servo.class, "outtakeClaw"));
        waitForStart();
        while (opModeIsActive()) {
            controller.setPIDF(p, i, d, 0);
            double power = controller.calculate(extensionRight.getCurrentPosition(), slideTarget);
            /*if (gamepad1.dpad_up) {
                extensionRight.setPower(1);
                extensionLeft.setPower(1);
            } else if (gamepad1.dpad_down) {
                extensionRight.setPower(-0.4);
                extensionLeft.setPower(-0.4);
            } else {
                extensionRight.setPower(0);
                extensionLeft.setPower(0);
            }*/
            extensionRight.setPower(power);
            extensionLeft.setPower(power);

            if (gamepad1.left_stick_button) {
                outtakeClaw.setPosition(0.9);
            }
            if (gamepad1.right_stick_button) {
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
            telemetry.addData("left_arm_position: ", outtakeArmLeft.getPosition());
            telemetry.addData("SlideError: ", slideTarget - extensionRight.getCurrentPosition());
            telemetry.addData("CurrentRight: ", extensionRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CurrentLeft: ", extensionLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftPower: ", extensionLeft.getPower());
            telemetry.addData("RightPower: ", extensionRight.getPower());
            telemetry.addData("slide pos: ", extensionRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
