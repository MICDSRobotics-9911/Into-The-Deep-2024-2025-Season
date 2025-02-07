package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.IntakeMacro;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SubmersibleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;

public class IntakeMacroToggleCommand extends ConditionalCommand {

        public IntakeMacroToggleCommand() {
        super(
                new IntakeMacro(),
                new SubmersibleCommand(),
                () -> RobotHardware.getInstance().intake.arm ==
                        IntakeSubsystem.ArmState.SUBMERSIBLE &&
                        RobotHardware.getInstance().intake.getClawState() == ClawState.CLOSED
        );
    }
}
