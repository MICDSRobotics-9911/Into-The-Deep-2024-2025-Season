package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class SampleScoreToggleCommand extends ConditionalCommand {
    public SampleScoreToggleCommand() {
        super(
                new SequentialCommandGroup(
                        new OuttakeClawCommand(ClawState.OPEN),
                        new WaitCommand(200),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.TRANSFER),
                        new WaitCommand(300),
                        new SlideCommand(OuttakeSubsystem.SlideState.RESET)
                ),
                new SequentialCommandGroup(
                        new OuttakeClawCommand(ClawState.CLOSED),
                        new SlideCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.UP)
                ),
                () -> RobotHardware.getInstance().outtake.slide == OuttakeSubsystem.SlideState.HIGH_BASKET
        );
    }
}
