package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class RobotHardware {
    private double voltage = 12.0;
    //public WebcamName camera;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public static RobotHardware instance = null;
    public boolean enabled;
    public Drivetrain drivetrain;
    private ArrayList<WSubsystem> subsystems;
    private IMU imu;
    public List<LynxModule> modules;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        // Add an if else detecting whether dashboard is running for telemetry
        this.telemetry = telemetry;

        // DRIVETRAIN
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        if (Globals.IS_USING_IMU) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(parameters);
            imu.resetYaw();
        }

        this.subsystems = new ArrayList<WSubsystem>();

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight, imu);
        addSubsystem(drivetrain);

        if (!Globals.IS_AUTO) {
            modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
        clearBulkCache();
    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }
}
