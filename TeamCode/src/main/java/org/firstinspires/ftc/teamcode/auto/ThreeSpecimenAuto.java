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
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.IntakeMacro;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenPreIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SubmersibleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.drivecommands.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.Drawing;
import org.firstinspires.ftc.teamcode.common.util.Pose;

import java.util.Locale;

@Config
@Autonomous(name="ThreeSpecimenAuto")
public class ThreeSpecimenAuto extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    //public static Pose startingPose = new Pose(10, 15, Math.toRadians(90));
    public static Pose zeroPose = new Pose(29, 8, Math.toRadians(180));
    public static Pose secondInBetweenPose = new Pose(29.8, 8, Math.toRadians(180));
    public static Pose inBetweenfirstPose = new Pose(18, 6, 0);
    public static Pose strafe = new Pose(18, -18, 0);
    public static Pose firstPose = new Pose(52, -22, 0);
    public static Pose firstPush = new Pose(10, -28, 0);
    public static Pose secondPose = new Pose(3, -38, 0);
    public static Pose inBetweenSecondPose = new Pose(52, -30, 0);
    public static Pose specimenIntake = new Pose(3, -38, 0);
    public static Pose inBetweenThirdPose = new Pose(15, -4, 0);
    public static Pose thirdPose = new Pose(10, -38, 0);
    public static Pose fourthPose = new Pose(29.8, 25, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Globals.IS_AUTO = true;
        Globals.threeSpec = true;

        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

        robot.init(hardwareMap);

        robot.setPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, Math.toRadians(180)));

        // for the heading to be 0, have the robot be facing the red submersible side,
        CommandScheduler.getInstance().schedule(
                new ResetCommand()
        );
        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket(true);
            dashboard.sendTelemetryPacket(packet);
            CommandScheduler.getInstance().run();
        }
        CommandScheduler.getInstance().reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new SpecimenIntakeCommand(),
                        new PositionCommand(zeroPose),
                        new PositionCommand(secondInBetweenPose, 0.7),
                        new ScoreSpecimenCommand(),
                        new PositionCommand(inBetweenfirstPose),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(strafe),
                        new PositionCommand(firstPose),
                        new PositionCommand(firstPush),
                        new PositionCommand(inBetweenSecondPose),
                        new PositionCommand(secondPose),
                        new PositionCommand(specimenIntake, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose),
                        new PositionCommand(new Pose(zeroPose.x + 0.3, zeroPose.y + 7, zeroPose.heading)),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 7, secondInBetweenPose.heading), 0.5),
                        new ScoreSpecimenCommand(),
                        new PositionCommand(inBetweenfirstPose),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(thirdPose),
                        new PositionCommand(specimenIntake, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose),
                        new PositionCommand(new Pose(zeroPose.x + 0.4, zeroPose.y + 15, zeroPose.heading)),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 15, secondInBetweenPose.heading)),
                        new ScoreSpecimenCommand(),
                        new PositionCommand(inBetweenfirstPose),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(thirdPose),
                        new PositionCommand(specimenIntake, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose),
                        new PositionCommand(new Pose(zeroPose.x + 0.5, zeroPose.y + 20, zeroPose.heading)),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 20, secondInBetweenPose.heading)),
                        new ScoreSpecimenCommand()
                )
        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            robot.read();
            robot.periodic();
            robot.write();
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),
                    pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("slide pos: ", robot.extensionRight.getCurrentPosition());
            telemetry.update();
            dashboard.sendTelemetryPacket(Drawing.drawRobot(robot.odo.getPosition()));
        }

        robot.kill();

    }


}
