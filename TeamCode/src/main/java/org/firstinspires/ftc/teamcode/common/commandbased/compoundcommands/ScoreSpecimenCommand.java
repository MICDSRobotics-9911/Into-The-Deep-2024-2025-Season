package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class ScoreSpecimenCommand extends SequentialCommandGroup {
    public ScoreSpecimenCommand() {
        super(
                new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_SCORING),
                new WaitCommand(300),
                new OuttakeClawCommand(ClawState.OPEN)
        );
        Globals.stopIntaking();
    }
}
