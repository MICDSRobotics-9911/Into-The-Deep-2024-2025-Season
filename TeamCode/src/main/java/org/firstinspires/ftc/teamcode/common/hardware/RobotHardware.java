package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class RobotHardware {
    private double voltage = 12.0;
    //public WebcamName camera;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public static RobotHardware instance = null;
    public boolean enabled;

    public MecanumDrivetrain drivetrain;
    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    private GoBildaPinpointDriver odo;
    public CachingDcMotorEx frontLeft;
    public CachingDcMotorEx frontRight;
    public CachingDcMotorEx backLeft;
    public CachingDcMotorEx backRight;

    public CachingServo intakeClaw;
    public CachingServo claw4bPivot;
    public CachingServo clawPivot;

    public CachingDcMotorEx extensionMotor;
    public CachingDcMotorEx extensionMotor2;

    public CachingServo outtakeClaw;
    public CachingServo outtakeClawPivot;
    public CachingServo outtakeClawPivot2;

    public HashMap<Sensors.SensorType, Object> values;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();
        // Add an if else detecting whether dashboard is running for telemetry
        this.telemetry = telemetry;

        values.put(Sensors.SensorType.EXTENSION_ENCODER, 0.0);
        values.put(Sensors.SensorType.PINPOINT, new Pose2D(
                DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
        );

        // DRIVETRAIN
        frontLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backLeft"));
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "backRight"));
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // X Pod Offset: 24.07432
        // Y Pod Offset: 62.37545
        odo.setOffsets(24.07432, 62.37545);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber()))
                CONTROL_HUB = m;
        }

        // Subsystems
        drivetrain = new MecanumDrivetrain();
        intake = new IntakeSubsystem();
        outtake = new OuttakeSubsystem();


        //camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        // Read all hardware devices here
        // 0 is a placeholder until i plug in the slide motor encoders
        values.put(Sensors.SensorType.EXTENSION_ENCODER, 0);
        values.put(Sensors.SensorType.PINPOINT, odo.getPosition());
    }

    public void write() {
        outtake.write();
        intake.write();
        drivetrain.write();
    }

    public void periodic() {
        intake.periodic();
        outtake.periodic();
        drivetrain.periodic();
        clearBulkCache();
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
        //CONTROL_HUB.clearBulkCache();
    }

    public double getVoltage() {
        return voltage;
    }

    public void kill() {
        instance = null;
    }

}
