package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@Disabled
@Config
@TeleOp(name="SlideTest")
public class SlideTest extends LinearOpMode {
    private DcMotorEx extensionLeft;
    private DcMotorEx extensionRight;
    private PIDFController controller;
    public static double kP = 0;
    public static double kD = 0;

    public static double leftPower = 0.5;
    public static double target = 2000;
    public static double kF = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDFController(kP, 0, kD, 0);
        extensionLeft = hardwareMap.get(DcMotorEx.class, "extensionLeft");
        extensionRight = hardwareMap.get(DcMotorEx.class, "extensionRight");
        extensionRight.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean pid = false;

        Gamepad currentGamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        waitForStart();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad1);
            currentGamepad.copy(gamepad1);
            int ticks = extensionLeft.getCurrentPosition();
            double pidPower = controller.calculate(ticks, target);
            pidPower += ticks * kF;

            if (currentGamepad.a && !lastGamepad.a) {
                pid = true;
            }

            if (gamepad1.dpad_up) {
                extensionLeft.setPower(leftPower);
                extensionRight.setPower(leftPower);
                pid = false;
            } else if (gamepad1.dpad_down) {
                extensionLeft.setPower(-leftPower);
                extensionRight.setPower(-leftPower);
                pid = false;
            } else if (pid) {
                extensionLeft.setPower(pidPower);
                extensionRight.setPower(pidPower);
            } else {
                extensionLeft.setPower(0);
                extensionRight.setPower(0);
            }
            telemetry.addData("LeftPower: ", leftPower);
            telemetry.update();
        }
    }
}
