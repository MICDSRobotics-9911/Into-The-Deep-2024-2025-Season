package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.CoaxialCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.LinkageCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class ResetCommand extends SequentialCommandGroup {
    public ResetCommand() {
        super(
                /* sigma sigma boy wrote this code */
                new IntakeArmCommand(IntakeSubsystem.ArmState.RESET),
                new OuttakeClawCommand(ClawState.CLOSED),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.RESET),
                new SlideCommand(OuttakeSubsystem.SlideState.RESET),
                new IntakeClawCommand(ClawState.CLOSED),
                new TurretCommand(IntakeSubsystem.TurretState.PERPENDICULAR),
                new ConditionalCommand(
                        new LinkageCommand(IntakeSubsystem.PivotState.RETRACT),
                        new LinkageCommand(IntakeSubsystem.PivotState.EXTEND),
                        () -> Globals.IS_AUTO
                ),
                new LinkageCommand(IntakeSubsystem.PivotState.RETRACT),
                new WaitCommand(200),
                new CoaxialCommand(IntakeSubsystem.CoaxialState.RESET)
        );
    }
}
