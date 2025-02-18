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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commandbased.TurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.IntakeMacro;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SubmersibleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.drivecommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.togglecommands.SampleScoreToggleCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Drawing;
import org.firstinspires.ftc.teamcode.common.util.Pose;

import java.util.Locale;

@Config
@Autonomous(name="FourSampleAutoNormalized\uD83D\uDE20")
public class FourSampleAutoNormalized extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    //public static Pose startingPose = new Pose(10, 15, Math.toRadians(90));
    public static Pose zeroPose = new Pose(-24, 9.5, Math.toRadians(45));
    public static Pose inBetweenfirstPose = new Pose(-16.5, 9.8, Math.toRadians(90));
    public static Pose firstPose = new Pose(-27, 9.7, Math.toRadians(90));
    public static Pose inBetweenSecondPose = new Pose(0, 10, Math.toRadians(45));
    public static Pose secondPose = new Pose(-10.5, 36, Math.toRadians(180));
    public static Pose thirdPose = new Pose(-26, 7, Math.toRadians(45));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Globals.IS_AUTO = true;
        Globals.threeSpec = false;


        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        Globals.RESET_ENCODER = true;
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
                        new SampleScoreToggleCommand(),
                        new WaitCommand(500),
                        new PositionCommand(zeroPose, 0.5),
                        new SampleScoreToggleCommand(),
                        new TurretCommand(IntakeSubsystem.TurretState.INTAKE),
                        new SubmersibleCommand(),
                        new PositionCommand(inBetweenfirstPose, 0.5, 0.5, 0.5),
                        new WaitCommand(300),
                        new IntakeMacro(),
                        new WaitCommand(400),
                        new TransferSampleCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                        new SampleScoreToggleCommand(),
                        new WaitCommand(1200),
                        new PositionCommand(zeroPose),
                        new SampleScoreToggleCommand(),
                        new TurretCommand(IntakeSubsystem.TurretState.INTAKE),
                        new SubmersibleCommand(),
                        new PositionCommand(firstPose, 0.5, 0.5, 0.5),
                        new WaitCommand(300),
                        new IntakeMacro(),
                        new WaitCommand(400),
                        // TODO: REMOVE WAIT BELOW
                        new TransferSampleCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                        new SampleScoreToggleCommand(),
                        new WaitCommand(1200),
                        new PositionCommand(thirdPose),
                        new SampleScoreToggleCommand(),
                        //new PositionCommand(inBetweenSecondPose, 0.9, 10, 10),
                        new PositionCommand(secondPose, 0.5, 0.5, 0.5),
                        new SubmersibleCommand(),
                        new TurretCommand(IntakeSubsystem.TurretState.PERPENDICULAR),
                        new WaitCommand(600),
                        new IntakeMacro(),
                        new WaitCommand(5000),
                        new WaitCommand(400),
                        new TransferSampleCommand(OuttakeSubsystem.SlideState.HIGH_BASKET),
                        new SampleScoreToggleCommand(),
                        new WaitCommand(1200),
                        new PositionCommand(zeroPose),
                        new SampleScoreToggleCommand()
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

        Globals.RESET_ENCODER = false;

        robot.kill();

    }


}
