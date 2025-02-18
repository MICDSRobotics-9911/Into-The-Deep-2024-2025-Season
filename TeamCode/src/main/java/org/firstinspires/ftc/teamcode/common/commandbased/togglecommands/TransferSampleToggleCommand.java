package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class TransferSampleToggleCommand extends ConditionalCommand {
    public TransferSampleToggleCommand(RobotHardware robot) {
        super(
                new SampleScoreToggleCommand(),
                new TransferSampleCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                () -> robot.outtake.claw == ClawState.CLOSED && robot.outtake.slide == OuttakeSubsystem.SlideState.RESET
                && robot.outtake.arm == OuttakeSubsystem.PivotState.SCORING && robot.intake.arm == IntakeSubsystem.ArmState.RESET
                && robot.intake.getPivotState() == IntakeSubsystem.PivotState.TRANSFER
        );

    }
}
