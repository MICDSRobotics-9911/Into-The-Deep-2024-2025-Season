package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.SpecimenPreIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbased.drivecommands.PositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Drawing;
import org.firstinspires.ftc.teamcode.common.util.Pose;

import java.util.Locale;

@Disabled
@Config
@Autonomous(name="FourSpecimenAuto\uD83D\uDC40")
public class FourSpecimenAuto extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;
    //public static Pose startingPose = new Pose(10, 15, Math.toRadians(90));
    public static Pose zeroPose = new Pose(-22, 8, 0);
    public static Pose secondInBetweenPose = new Pose(-22, 8, 0);
    public static Pose inBetweenfirstPose = new Pose(-18, -6, Math.toRadians(180));
    public static Pose strafe = new Pose(-16, 15, Math.toRadians(180));
    public static Pose firstPose = new Pose(-52, 25, Math.toRadians(180));
    public static Pose firstPush = new Pose(-10, 30, Math.toRadians(180));
    public static Pose secondPose = new Pose(-3, 40, Math.toRadians(180));
    public static Pose inBetweenSecondPose = new Pose(-52, 30, Math.toRadians(180));
    public static Pose specimenIntake = new Pose(-3, 40, Math.toRadians(180));
    public static Pose inBetweenThirdPose = new Pose(-15, 0, 0);
    public static Pose thirdPose = new Pose(-3, 25, Math.toRadians(180));
    public static Pose fourthPose = new Pose(-3, 25, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Globals.IS_AUTO = true;
        Globals.threeSpec = false;

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
                        //new PositionCommand(inBetweenfirstPose, 0.9,10, 20),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(strafe,0.9, 10, 10)/*,
                        new PositionCommand(firstPose, 0.9, 10, 10),
                        new PositionCommand(firstPush, 0.9, 6, 3),
                        new PositionCommand(inBetweenSecondPose, 0.9, 10, 10),
                        new PositionCommand(secondPose, 0.9),
                        new PositionCommand(specimenIntake, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose, 0.9, 10, 10),
                        new PositionCommand(new Pose(zeroPose.x, zeroPose.y - 5, zeroPose.heading), 0.9, 6, 6),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 7, secondInBetweenPose.heading), 0.5, 2, 3),
                        new ScoreSpecimenCommand(),
                        new PositionCommand(thirdPose, 0.9, 10, 10),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(thirdPose, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose, 0.9, 10, 10),
                        new PositionCommand(new Pose(zeroPose.x, zeroPose.y - 10, zeroPose.heading), 0.9, 6, 6),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 15, secondInBetweenPose.heading), 0.5, 2, 3),
                        new ScoreSpecimenCommand(),
                        new PositionCommand(fourthPose, 0.9, 10, 10),
                        new SpecimenPreIntakeCommand(),
                        new PositionCommand(fourthPose, 0.7),
                        new SpecimenIntakeCommand(),
                        new PositionCommand(inBetweenThirdPose, 0.9, 10, 10),
                        new PositionCommand(new Pose(zeroPose.x, zeroPose.y - 15, zeroPose.heading), 0.9, 6, 6),
                        new PositionCommand(new Pose(secondInBetweenPose.x, secondInBetweenPose.y + 20, secondInBetweenPose.heading), 0.5, 2, 3),
                        new ScoreSpecimenCommand()*/
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
