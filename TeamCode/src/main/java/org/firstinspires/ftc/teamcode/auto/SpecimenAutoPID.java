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
import org.firstinspires.ftc.teamcode.common.commandbased.SlideCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenIntakeCommand;
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
@Autonomous(name="SpecimenAutoPID")
public class SpecimenAutoPID extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    public static Pose zeroPose = new Pose(29.9, 15, Math.toRadians(180));
    public static Pose inBetweenPose = new Pose(25, 10, Math.toRadians(180));
    public static Pose secondInBetweenPose = new Pose(29.8, 20, Math.toRadians(180));
    public static Pose inBetweenfirstPose = new Pose(6, -31, Math.toRadians(180));
    public static Pose firstPose = new Pose(6, -31, 0);
    public static Pose secondPose = new Pose(2.3, -31, 0);
    public static Pose inBetweenSecondPose = new Pose(7, -31, 0);
    public static Pose thirdPose = new Pose(25, 25, 0);
    public static Pose fourthPose = new Pose(29.8, 25, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);

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
                        new SpecimenIntakeCommand(),
                        new PositionCommand(zeroPose),
                        new PositionCommand(inBetweenPose),
                        new PositionCommand(secondInBetweenPose, 0.5)
                        //new ScoreSpecimenCommand(),
                        /*new PositionCommand(inBetweenfirstPose),
                        new PositionCommand(new Pose(firstPose.x, firstPose.y, Math.toRadians(180))),
                        new PositionCommand(firstPose),
                        new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_INTAKE),
                        new WaitCommand(1000),
                        new PositionCommand(secondPose, 0.3),
                        new OuttakeClawCommand(ClawState.CLOSED),
                        new WaitCommand(1000),
                        new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_OUTTAKE),
                        new PositionCommand(new Pose(10, 20, 0)),
                        new PositionCommand(inBetweenPose),
                        //new ManualSlideCommand(2000),

                        //new WaitCommand(4000),
                        new PositionCommand(new Pose(secondInBetweenPose.x,
                                secondInBetweenPose.y - 10, secondInBetweenPose.heading), 0.5),
                        //new SlideCommand(OuttakeSubsystem.SlideState.SPECIMEN_SCORING),
                        //new WaitCommand(700),
                        //new OuttakeClawCommand(ClawState.OPEN)\
                        new WaitCommand(2000),
                        new ScoreSpecimenCommand(),
                        new WaitCommand(3000),
                        new OuttakeArmCommand(OuttakeSubsystem.PivotState.UP)*/
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
            telemetry.addData("slide pos: ", robot.extensionRight.getCurrentPosition());
            telemetry.update();
            dashboard.sendTelemetryPacket(Drawing.drawRobot(robot.odo.getPosition()));
        }

        robot.kill();

    }


}
