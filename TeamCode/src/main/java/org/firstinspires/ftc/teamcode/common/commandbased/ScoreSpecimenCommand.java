package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class ScoreSpecimenCommand extends SequentialCommandGroup {
    public ScoreSpecimenCommand() {
        super(
                new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_SCORING),
                new OuttakeClawCommand(ClawState.OPEN)
        );
    }
}
