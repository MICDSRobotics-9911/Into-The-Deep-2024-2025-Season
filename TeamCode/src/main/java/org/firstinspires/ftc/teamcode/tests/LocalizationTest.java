package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Drawing;
import org.firstinspires.ftc.teamcode.common.util.Pose;

@TeleOp(name="LocalizationTest")
public class LocalizationTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private FtcDashboard dashboard;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.stopIntaking();
        Globals.stopScoring();

        robot.init(hardwareMap);

        robot.read();
        robot.setPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            dashboard.sendTelemetryPacket(new TelemetryPacket(true));
            telemetry.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        robot.drivetrain.set(
                new Pose(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                ), 0
        );

        dashboard.sendTelemetryPacket(Drawing.drawRobot(robot.getPose()));
        telemetry.addLine(robot.drivetrain.toString());
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
