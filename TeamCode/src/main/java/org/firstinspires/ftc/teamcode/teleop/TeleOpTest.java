package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="TeleOpTest")
public class TeleOpTest extends OpMode {
    private RobotHardware robot;
    private double loopTime = 0.0;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Globals.IS_AUTO = false;
        Globals.retract();
        Globals.IS_USING_IMU = false;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);
        robot.drivetrain.read();
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addLine(robot.drivetrain.toString());
        double loop = System.nanoTime();
        loopTime = loop;
        telemetry.addLine(robot.drivetrain.toString());
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        telemetry.update();
    }

    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = power * cos / max + turn;
        double rightFront = power * sin / max - turn;
        double leftBack = power * sin / max + turn;
        double rightBack = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftBack /= power + Math.abs(turn);
            rightBack /= power + Math.abs(turn);
        }
        robot.frontLeft.setPower(leftFront);
        robot.frontRight.setPower(rightFront);
        robot.backLeft.setPower(leftBack);
        robot.backRight.setPower(rightBack);
    }
}
