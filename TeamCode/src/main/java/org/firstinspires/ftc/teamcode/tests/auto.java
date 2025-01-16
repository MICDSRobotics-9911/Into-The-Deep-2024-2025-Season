package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Pose;

@Autonomous(name="autodrivestraight")
public class auto extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.setPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.set(1, 0, 0, 0);
            robot.periodic();
            robot.write();
            robot.read();
            telemetry.addLine(robot.drivetrain.toString());
            telemetry.update();
            //robot.drivetrain.set(0, 0, 0, 0);
        }
    }
}
