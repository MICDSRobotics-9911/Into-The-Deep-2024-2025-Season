package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        robot.setPose(new Pose());
        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.set(0, 1, 0, 0);
            robot.periodic();
            robot.write();
            robot.read();
            telemetry.addLine(robot.drivetrain.toString());
            telemetry.update();
            //robot.drivetrain.set(0, 0, 0, 0);
        }
    }
}
