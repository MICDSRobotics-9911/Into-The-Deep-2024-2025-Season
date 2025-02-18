package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem.SlideState;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbased.CoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetEncoderCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenPreIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.IntakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.IntakeMacroToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.LinkageToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.OuttakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.OuttakeArmToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.SampleScoreToggleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.SpecimenToggle;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.TransferSampleToggleCommand;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
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
    private TriggerReader rightTrigger;
    private TriggerReader leftTrigger;

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
        robot.init(hardwareMap);

        robot.read();
        CommandScheduler.getInstance().schedule(
                new ResetCommand()
        );
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();
        initializeButtons();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        if (-gamepad2.left_stick_y > 0.1) {
            robot.outtake.usePIDF = false;
            robot.extensionLeft.setPower(1);
            robot.extensionRight.setPower(1);
            robot.outtake.slideTarget = robot.outtake.getSlidePosition();
        } else if (-gamepad2.left_stick_y < -0.1) {
            robot.outtake.usePIDF = false;
            robot.extensionLeft.setPower(-0.4);
            robot.extensionRight.setPower(-0.4);
            robot.outtake.slideTarget = robot.outtake.getSlidePosition();
        } else {
            robot.outtake.usePIDF = true;
        }

        if (rightTrigger.wasJustPressed()) {
            new IntakeClawToggleCommand(robot);
        }

        if (leftTrigger.wasJustPressed()) {
            new OuttakeClawToggleCommand(robot);
        }


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
        telemetry.addData("OuttakeClaw: ", robot.outtake.getClawState());
        telemetry.addData("IntakeClaw: ", robot.intake.getClawState());
        loopTime = loop;
        telemetry.update();
    }


    public void initializeButtons() {
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SampleScoreToggleCommand());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SequentialCommandGroup(
                        new SlideCommand(SlideState.RESET),
                        new OuttakeClawCommand(ClawState.OPEN)
                ));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SpecimenToggle());
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new TransferSampleCommand(SlideState.HIGH_BASKET));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeMacroToggleCommand());
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SpecimenPreIntakeCommand());
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LinkageToggleCommand(robot));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new IntakeArmCommand(IntakeSubsystem.ArmState.RESET));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new TurretCommand(IntakeSubsystem.TurretState.DECREMENT));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new TurretCommand(IntakeSubsystem.TurretState.INCREMENT));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new IntakeClawToggleCommand(robot));
        rightTrigger = new TriggerReader(
                gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        leftTrigger = new TriggerReader(
                gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> robot.intake.setCoaxialOffset(robot.intake.coaxialPos)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new CoaxialCommand(IntakeSubsystem.CoaxialState.INCREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new CoaxialCommand(IntakeSubsystem.CoaxialState.DECREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new IntakeClawToggleCommand(robot));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new OuttakeClawToggleCommand(robot));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new IntakeArmCommand(IntakeSubsystem.ArmState.RESET));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ResetEncoderCommand(robot));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new LinkageCommand(IntakeSubsystem.PivotState.INCREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LinkageCommand(IntakeSubsystem.PivotState.DECREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new IntakeArmCommand(IntakeSubsystem.ArmState.DECREMENT));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeArmCommand(IntakeSubsystem.ArmState.INCREMENT));
    }
}
