package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class SampleScoreToggleCommand extends ConditionalCommand {
    public SampleScoreToggleCommand() {
        super(
                new OuttakeClawCommand(ClawState.OPEN),
                new SequentialCommandGroup(
                        new SlideCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.SCORING)
                ),
                () -> RobotHardware.getInstance().outtake.slide == OuttakeSubsystem.SlideState.HIGH_BASKET
        );
    }
}
