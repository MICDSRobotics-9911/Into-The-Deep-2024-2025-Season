package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class OuttakeClawToggleCommand extends ConditionalCommand {
    public OuttakeClawToggleCommand(RobotHardware robot) {
        super(
                new OuttakeClawCommand(ClawState.OPEN),
                new OuttakeClawCommand(ClawState.CLOSED),
                () -> (robot.outtake.getClawState() == (ClawState.CLOSED))
        );
    }
}
