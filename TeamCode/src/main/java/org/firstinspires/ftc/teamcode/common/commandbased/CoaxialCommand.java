package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

public class CoaxialCommand extends ConditionalCommand {
    public CoaxialCommand(IntakeSubsystem.CoaxialState state, RobotHardware robot) {
        super(
                new InstantCommand(),
                new InstantCommand(() -> robot.intake.updateState(state)),
                () -> robot.intakeArmLeft.getPosition() > 0.8
        );
    }
}
