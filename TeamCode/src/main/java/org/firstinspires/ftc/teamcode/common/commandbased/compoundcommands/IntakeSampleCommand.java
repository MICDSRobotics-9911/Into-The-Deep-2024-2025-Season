package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbased.CoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand() {
        super(
                new LinkageCommand(IntakeSubsystem.PivotState.EXTEND),
                new IntakeArmCommand(IntakeSubsystem.ArmState.INTAKE),
                new TurretCommand(IntakeSubsystem.TurretState.INTAKE),
                new IntakeClawCommand(ClawState.OPEN),
                new CoaxialCommand(IntakeSubsystem.CoaxialState.INTAKE, RobotHardware.getInstance())
        );
        Globals.stopScoring();
    }
}
