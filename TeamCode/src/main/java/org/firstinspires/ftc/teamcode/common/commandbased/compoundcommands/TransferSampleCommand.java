package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbased.CoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new LinkageCommand(IntakeSubsystem.PivotState.RETRACT),
                new IntakeArmCommand(IntakeSubsystem.ArmState.TRANSFER1),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.TRANSFER),
                new CoaxialCommand(IntakeSubsystem.CoaxialState.TRANSFER,
                        RobotHardware.getInstance()),
                new TurretCommand(IntakeSubsystem.TurretState.TRANSFER),
                new IntakeArmCommand(IntakeSubsystem.ArmState.TRANSFER1),
                new IntakeArmCommand(IntakeSubsystem.ArmState.TRANSFER2),
                new OuttakeClawCommand(ClawState.CLOSED),
                new IntakeClawCommand(ClawState.OPEN),
                new IntakeArmCommand(IntakeSubsystem.ArmState.TRANSFER1)
        );
    }
}
