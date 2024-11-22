package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;

public class SlideCommand extends InstantCommand {
    public SlideCommand(OuttakeSubsystem.SlideState state) {
        super(
                () -> RobotHardware.getInstance().outtake.updateState(state)
        );
    }
}
