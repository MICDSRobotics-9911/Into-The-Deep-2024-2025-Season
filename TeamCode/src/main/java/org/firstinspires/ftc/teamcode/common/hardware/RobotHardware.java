package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

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
    public MecanumDrivetrain drivetrain;
    private ArrayList<WSubsystem> subsystems;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    private GoBildaPinpointDriver odo;
    public CachingDcMotorEx frontLeft;
    public CachingDcMotorEx frontRight;
    public CachingDcMotorEx backLeft;
    public CachingDcMotorEx backRight;

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
        frontLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: Test this
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: Test this
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeft"));
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: Test this
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRight"));
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: Test this
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // TODO: Test this
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }

        subsystems = new ArrayList<>();
        drivetrain = new MecanumDrivetrain();
        addSubsystem(drivetrain);


        //camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        drivetrain.write();
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
