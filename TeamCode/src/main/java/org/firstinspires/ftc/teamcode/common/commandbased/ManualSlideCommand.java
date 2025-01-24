package org.firstinspires.ftc.teamcode.common.commandbased;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ManualSlideCommand extends CommandBase {

    public static int target = 0;
    public static double DEAD_MS = 2500;
    public static double STABLE_MS = 100;
    public static double tolerance = 100;
    private RobotHardware robot = RobotHardware.getInstance();
    private ElapsedTime timer;
    private ElapsedTime stable;
    private int error;

    public ManualSlideCommand(int target) {
        this.target = target;
    }

    @Override
    public void execute() {
        error = target - robot.extensionRight.getCurrentPosition();

        if (error > 0) {
            robot.extensionRight.setPower(1);
            robot.extensionLeft.setPower(1);
        } else {
            robot.extensionRight.setPower(-1);
            robot.extensionLeft.setPower(-1);
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(error) > tolerance)
            stable.reset();
        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
    }

    @Override
    public void end(boolean interrupted) {
        robot.extensionRight.setPower(0);
        robot.extensionLeft.setPower(0);
    }
}
