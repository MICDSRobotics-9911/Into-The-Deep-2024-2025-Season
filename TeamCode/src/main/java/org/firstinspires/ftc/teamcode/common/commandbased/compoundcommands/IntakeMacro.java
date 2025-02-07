package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class IntakeMacro extends SequentialCommandGroup {
    public IntakeMacro() {
        super(
                new IntakeArmCommand(IntakeSubsystem.ArmState.INTAKE),
                new WaitCommand(400),
                new IntakeClawCommand(ClawState.OPEN)
        );
    }
}
