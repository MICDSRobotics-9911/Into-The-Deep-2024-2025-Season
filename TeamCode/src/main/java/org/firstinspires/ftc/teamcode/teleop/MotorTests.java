package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="MotorTests")
public class MotorTests extends OpMode {
    private RobotHardware robot;

    @Override
    public void init() {
        Globals.IS_AUTO = true;
        Globals.retract();
        Globals.IS_USING_IMU = false;
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap, telemetry);
        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    /**
     * Xbox/PS4 Button - Motor
     *   X / ▢         - Front Left
     *   Y / Δ         - Front Right
     *   B / O         - Rear  Right
     *   A / X         - Rear  Left
     *                                    The buttons are mapped to match the wheels spatially if you
     *                                    were to rotate the gamepad 45deg°. x/square is the front left
     *                    ________        and each button corresponds to the wheel as you go clockwise
     *                   / ______ \
     *     ------------.-'   _  '-..+              Front of Bot
     *              /   _  ( Y )  _  \                  ^
     *             |  ( X )  _  ( B ) |     Front Left   \    Front Right
     *        ___  '.      ( A )     /|       Wheel       \      Wheel
     *      .'    '.    '-._____.-'  .'       (x/▢)        \     (Y/Δ)
     *     |       |                 |                      \
     *      '.___.' '.               |          Rear Left    \   Rear Right
     *               '.             /             Wheel       \    Wheel
     *                \.          .'              (A/X)        \   (B/O)
     *                  \________/
     */
    @Override
    public void loop() {
        double power = 0.5;
        if (gamepad1.x)
            robot.drivetrain.wheelPowers[0] = power;
        else if (gamepad1.y)
            robot.drivetrain.wheelPowers[1] = power;
        else if (gamepad1.a)
            robot.drivetrain.wheelPowers[2] = power;
        else if (gamepad1.b)
            robot.drivetrain.wheelPowers[3] = power;
        else
            robot.reset();

        robot.write();
        telemetry.addLine(robot.drivetrain.toString());
        telemetry.update();
    }
}
