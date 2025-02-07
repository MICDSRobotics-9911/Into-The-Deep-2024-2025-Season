package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class SpecimenPreIntakeCommand extends SequentialCommandGroup {
    public SpecimenPreIntakeCommand() {
        super(
                new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_INTAKE),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.SCORING),
                new OuttakeClawCommand(ClawState.OPEN)
        );
    }
}
