package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class IntakeArmCommand extends InstantCommand {
    public IntakeArmCommand(IntakeSubsystem.ArmState state) {
        super(
                () -> RobotHardware.getInstance().intake.updateState(state)
        );
    }
}
