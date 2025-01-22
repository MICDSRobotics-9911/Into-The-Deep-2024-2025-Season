package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem.SlideState;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbased.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.IntakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.LinkageToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.OuttakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.OuttakeArmToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Pose;

@TeleOp(name="Solo TeleOp\uD83D\uDC80")
public class SoloTeleOp extends CommandOpMode {

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
        initializeButtons();
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

        /*if (gamepad1.dpad_up) {
            robot.extensionLeft.setPower(0.7);
            robot.extensionRight.setPower(0.7);
        } else if (gamepad1.dpad_down) {
            robot.extensionLeft.setPower(-0.3);
            robot.extensionRight.setPower(-0.3);
        } else {
            robot.extensionLeft.setPower(0);
            robot.extensionRight.setPower(0);
        }*/


        robot.drivetrain.set(
                new Pose(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                        // MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)
                ), 0
        );

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("SlideError: ", robot.outtake.slideTarget -
                robot.extensionRight.getCurrentPosition());
        telemetry.addData("Ticks: ", robot.extensionRight.getCurrentPosition());
        loopTime = loop;
        telemetry.update();
    }


    public void initializeButtons() {
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeClawToggleCommand(robot));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new OuttakeClawToggleCommand(robot));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SlideCommand(SlideState.HIGH_BASKET));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideCommand(SlideState.SPECIMEN_OUTTAKE));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(new ScoreSpecimenCommand());
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SlideCommand(SlideState.SPECIMEN_INTAKE));
        //gamepadEx.getGamepadButton(GamepadKeys.Button.A)
        //                .whenPressed(new )
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new OuttakeArmToggleCommand(robot));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LinkageToggleCommand(robot));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new OuttakeArmCommand(OuttakeSubsystem.PivotState.INCREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new OuttakeArmCommand(OuttakeSubsystem.PivotState.DECREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SlideCommand(SlideState.INCREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideCommand(SlideState.DECREMENT));
    }
}
