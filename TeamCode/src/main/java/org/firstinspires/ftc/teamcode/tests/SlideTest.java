package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp(name="SlideTest")
public class SlideTest extends LinearOpMode {
    private DcMotorEx extensionLeft;
    private PIDFController controller;
    public static double kP = 0;
    public static double kD = 0;

    public static double leftPower = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDFController(kP, 0, kD, 0);
        extensionLeft = hardwareMap.get(DcMotorEx.class, "extensionLeft");
        extensionLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                extensionLeft.setPower(leftPower);
            } else if (gamepad1.dpad_down) {
                extensionLeft.setPower(-leftPower);
            } else {
                extensionLeft.setPower(0);
            }
            telemetry.addData("LeftPower: ", leftPower);
            telemetry.update();
        }
    }
}
