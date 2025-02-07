package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;

import java.util.function.BooleanSupplier;

public class SpecimenToggle extends ConditionalCommand {
    public SpecimenToggle() {
        super(
                new ScoreSpecimenCommand(),
                new SpecimenIntakeCommand(),
                () -> RobotHardware.getInstance().outtake.slide ==
                        OuttakeSubsystem.SlideState.SPECIMEN_OUTTAKE
        );
    }
}
