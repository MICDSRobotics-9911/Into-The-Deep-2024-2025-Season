package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class SpecimenIntakeCommand extends SequentialCommandGroup {
    public SpecimenIntakeCommand() {
        super(
                new OuttakeClawCommand(ClawState.CLOSED),
                new WaitCommand(300),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.SCORING),
                new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_OUTTAKE)
        );
    }
}
