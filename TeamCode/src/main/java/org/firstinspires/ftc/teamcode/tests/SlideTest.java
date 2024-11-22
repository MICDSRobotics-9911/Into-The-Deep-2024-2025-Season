package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="SlideTest")
public class SlideTest extends LinearOpMode {
    private DcMotorEx extensionLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        extensionLeft = hardwareMap.get(DcMotorEx.class, "extensionLeft");
        double leftPower = 0.2;
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
