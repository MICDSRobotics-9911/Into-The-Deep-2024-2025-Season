package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class IntakeClawToggleCommand extends ConditionalCommand {
    public IntakeClawToggleCommand(RobotHardware robot) {
        super(
                new IntakeClawCommand(ClawState.OPEN),
                new IntakeClawCommand(ClawState.CLOSED),
                () -> (robot.intake.getClawState() == (ClawState.CLOSED))
        );
    }
}
