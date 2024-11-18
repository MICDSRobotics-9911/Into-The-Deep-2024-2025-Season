package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbased.drivecommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Pose;

@Config
@Autonomous(name="P2P")
public class PositionCommandTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot.init(hardwareMap);

        robot.setPose(new Pose());

        // for the heading to be 0, have the robot be facing the red submersible side,
        Pose testPose = new Pose(-30, -34, 0);

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.update();
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(testPose)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.clearBulkCache();
            robot.read();
            robot.periodic();
            robot.write();
            telemetry.addLine("targetPose: " + testPose);
            telemetry.addLine("currentPose: " + robot.getPose());
            telemetry.addData("xError: ", robot.getPose().x - testPose.x);
            telemetry.addData("yError: ", robot.getPose().y - testPose.y);
            telemetry.addData("hError: ", robot.getPose().heading - testPose.heading);
            telemetry.update();
            if (!testPose.equals(new Pose())) {
                testPose = new Pose();
            } else {
                testPose = new Pose(-30, -34, 0);
            }
        }

        robot.kill();

    }


}
