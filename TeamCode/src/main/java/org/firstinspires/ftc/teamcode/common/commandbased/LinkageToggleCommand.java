package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class LinkageToggleCommand extends ConditionalCommand  {
    public LinkageToggleCommand(RobotHardware robot) {
        super(
                new LinkageCommand(IntakeSubsystem.PivotState.EXTEND),
                new LinkageCommand(IntakeSubsystem.PivotState.RETRACT),
                () -> (robot.intake.getPivotState() == IntakeSubsystem.PivotState.RETRACT)
        );
    }
}
