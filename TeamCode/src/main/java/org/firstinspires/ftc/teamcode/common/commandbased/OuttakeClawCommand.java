package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class OuttakeClawCommand extends InstantCommand {
    public OuttakeClawCommand(ClawState state) {
        super(
                () -> RobotHardware.getInstance().outtake.updateState(state)
        );
    }
}
