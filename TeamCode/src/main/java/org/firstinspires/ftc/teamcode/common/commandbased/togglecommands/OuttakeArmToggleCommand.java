package org.firstinspires.ftc.teamcode.common.commandbased.togglecommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;

public class OuttakeArmToggleCommand extends ConditionalCommand {
    public OuttakeArmToggleCommand(RobotHardware robot) {
        super(
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.SCORING),
                new OuttakeArmCommand(OuttakeSubsystem.PivotState.TRANSFER),
                () -> (robot.outtake.getArmState() == OuttakeSubsystem.PivotState.TRANSFER)
        );
    }
}
