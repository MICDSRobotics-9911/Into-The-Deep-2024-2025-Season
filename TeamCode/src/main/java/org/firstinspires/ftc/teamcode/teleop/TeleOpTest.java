package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import java.util.Objects;

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
        robot.init(hardwareMap, telemetry);
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
        robot.drivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addLine(robot.drivetrain.toString());
        double loop = System.nanoTime();
        loopTime = loop;
        robot.periodic();
        telemetry.addLine(robot.drivetrain.toString());
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        telemetry.update();
    }
}
