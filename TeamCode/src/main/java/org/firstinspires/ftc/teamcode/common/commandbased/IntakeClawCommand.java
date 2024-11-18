package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(ClawState state) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state)
        );
    }
}
