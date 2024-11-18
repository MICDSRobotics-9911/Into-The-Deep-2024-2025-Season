package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Pose;

@TeleOp(name="Solo TeleOp\uD83D\uDC80")
public class CommandTeleOp extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;
    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;

    private boolean extendIntake = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.stopIntaking();
        Globals.stopScoring();

        intake = robot.intake;
        outtake = robot.outtake;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new IntakeClawToggleCommand(robot));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(new OuttakeClawToggleCommand(robot));

        robot.init(hardwareMap);

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        if (gamepad1.dpad_right) {
            robot.linkageServo.setPosition(1);
        }
        if (gamepad1.dpad_left) {
            robot.linkageServo.setPosition(0);
        }
        if (gamepad1.dpad_up) {
            // TODO: Tune this
            robot.outtake.usePIDF = true;
            outtake.setTargetPosition(1000);
            robot.extension.setPower(outtake.getPID());
        } else if (gamepad1.dpad_down) {
            robot.outtake.usePIDF = true;
            outtake.setTargetPosition(0);
            robot.extension.setPower(outtake.getPID());
        }
        if (gamepad1.right_trigger > 0) {
            robot.outtake.usePIDF = false;
            robot.extension.setPower(gamepad1.right_trigger);
        }
        if (gamepad1.left_trigger > 0) {
            robot.outtake.usePIDF = false;
            robot.extension.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.x) {

        }
        if (gamepad1.y) {

        }




        robot.drivetrain.set(
                new Pose(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                        // MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)
                ), 0
        );

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
