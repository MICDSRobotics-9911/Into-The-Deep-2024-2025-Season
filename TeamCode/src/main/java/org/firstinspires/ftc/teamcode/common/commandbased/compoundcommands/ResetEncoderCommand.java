package org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ResetEncoderCommand extends SequentialCommandGroup {
    public ResetEncoderCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)),
                new InstantCommand(() -> robot.extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
        );
    }
}
