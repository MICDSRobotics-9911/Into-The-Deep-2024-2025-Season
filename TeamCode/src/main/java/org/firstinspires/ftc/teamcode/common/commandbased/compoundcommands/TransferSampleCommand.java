package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
                new CoaxialCommand(IntakeSubsystem.CoaxialState.TRANSFER),
                new TurretCommand(IntakeSubsystem.TurretState.TRANSFER),
                new IntakeArmCommand(IntakeSubsystem.ArmState.SUBMERSIBLE),
                new LinkageCommand(IntakeSubsystem.PivotState.RETRACT),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.TRANSFER),
                new OuttakeClawCommand(ClawState.OPEN),
                new WaitCommand(600),
                new IntakeArmCommand(IntakeSubsystem.ArmState.TRANSFER),
                new WaitCommand(300),
                //new IntakeArmCommand(IntakeSubsystem.ArmState.SUBMERSIBLE),
                new OuttakeClawCommand(ClawState.CLOSED),
                new WaitCommand(300),
                new IntakeClawCommand(ClawState.CLOSED)
                //new IntakeArmCommand(IntakeSubsystem.ArmState.RESET)
        );
    }
}
