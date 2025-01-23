package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.drivecommands.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.OuttakeClawToggleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.Drawing;
import org.firstinspires.ftc.teamcode.common.util.Pose;

import java.util.Locale;

@Config
@Autonomous(name="BlueSpecimenAuto")
public class BlueSpecimenAuto extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    public static Pose firstPose = new Pose(25, -31, 0);
    public static Pose secondPose = new Pose(3, -31, 0);
    public static Pose thirdPose = new Pose(25, 20, 0);
    public static Pose fourthPose = new Pose(35, 25, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        robot.init(hardwareMap);

        robot.setPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        // for the heading to be 0, have the robot be facing the red submersible side,

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket(true);
            dashboard.sendTelemetryPacket(packet);
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(firstPose),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.SCORING),
                        new WaitCommand(1000),
                        new PositionCommand(secondPose, 0.3),
                        new OuttakeClawCommand(ClawState.CLOSED),
                        new WaitCommand(1000),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.TRANSFER),
                        new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_OUTTAKE),
                        new PositionCommand(thirdPose)
                        //new PositionCommand(fourthPose)
                        //new ScoreSpecimenCommand()
                )
        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();
            /*telemetry.addData("xError: ", robot.getPose().getX(DistanceUnit.INCH) -
                    testPose.x);
            telemetry.addData("yError: ", robot.getPose().getY(DistanceUnit.INCH) -
                    testPose.y);
            telemetry.addData("hError: ", robot.getPose().
                    getHeading(AngleUnit.DEGREES) - Math.toDegrees(testPose.heading));*/
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),
                    pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
            dashboard.sendTelemetryPacket(Drawing.drawRobot(robot.odo.getPosition()));
        }

        robot.kill();

    }


}
