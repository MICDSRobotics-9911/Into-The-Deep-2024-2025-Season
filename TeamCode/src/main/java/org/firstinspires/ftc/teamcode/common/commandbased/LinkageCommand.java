package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class LinkageCommand extends InstantCommand {
    public LinkageCommand(IntakeSubsystem.PivotState state) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state)
        );
    }
}
