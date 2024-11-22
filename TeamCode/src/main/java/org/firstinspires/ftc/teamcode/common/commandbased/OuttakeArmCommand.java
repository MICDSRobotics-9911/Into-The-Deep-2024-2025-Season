package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;

public class OuttakeArmCommand extends InstantCommand {
    public OuttakeArmCommand(OuttakeSubsystem.PivotState state) {
        super(
                () -> RobotHardware.getInstance().outtake.updateState(state)
        );
    }
}
