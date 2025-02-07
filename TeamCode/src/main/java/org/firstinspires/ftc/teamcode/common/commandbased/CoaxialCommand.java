package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class CoaxialCommand extends InstantCommand {
    public CoaxialCommand(IntakeSubsystem.CoaxialState state) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state)
        );
    }
}
