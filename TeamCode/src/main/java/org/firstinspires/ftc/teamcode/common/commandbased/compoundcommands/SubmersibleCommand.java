package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.CoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class SubmersibleCommand extends SequentialCommandGroup {
    public SubmersibleCommand() {
        super(
                new IntakeArmCommand(IntakeSubsystem.ArmState.SUBMERSIBLE),
                new IntakeClawCommand(ClawState.CLOSED),
                new LinkageCommand(IntakeSubsystem.PivotState.EXTEND),
                new WaitCommand(200),
                new CoaxialCommand(IntakeSubsystem.CoaxialState.INTAKE)
        );
    }
}
