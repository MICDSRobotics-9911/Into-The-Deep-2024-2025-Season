package org.firstinspires.ftc.teamcode.common.hardware;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.commandbased.compoundcommands.ResetCommand;
import org.firstinspires.ftc.teamcode.common.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.LinkedMotors;
import org.firstinspires.ftc.teamcode.common.util.LinkedServos;
import org.firstinspires.ftc.teamcode.common.util.Pose;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver;

import java.util.HashMap;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class RobotHardware {
    private double voltage = 12.0;
    private ElapsedTime voltageTimer = new ElapsedTime();
    //public WebcamName camera;
    private HardwareMap hardwareMap;
    public static RobotHardware instance = null;
    public boolean enabled;

    public MecanumDrivetrain drivetrain;
    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    public GoBildaPinpointDriver odo;
    public CachingDcMotorEx frontLeft;
    public CachingDcMotorEx frontRight;
    public CachingDcMotorEx backLeft;
    public CachingDcMotorEx backRight;

    public CachingServo linkageServoRight;
    public CachingServo linkageServoLeft;
    public LinkedServos linkageServo;

    public CachingServo intakeClaw;
    public CachingServo turretClaw;
    public CachingServo intakeArmLeft;
    public CachingServo intakeArmRight;
    public LinkedServos intakeArm;

    public CachingDcMotorEx extensionLeft;
    public CachingDcMotorEx extensionRight;
    public LinkedMotors extension;

    public CachingServo outtakeClaw;
    public CachingServo outtakeArmLeft;
    public CachingServo outtakeArmRight;
    public LinkedServos outtakeArm;

    public HashMap<Sensors.SensorType, Object> values;

    private Pose2D robotPose;
    public ServoImplEx intakeCoaxial;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();
        // Add an if else detecting whether dashboard is running for telemetry

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

        // EXTENSION
        extensionRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionRight"));
        extensionRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (Globals.RESET_ENCODER) {
            extensionRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        extensionLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extensionLeft"));
        extensionLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // OUTTAKE
        outtakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmRight"));
        outtakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "outtakeArmLeft"));
        outtakeArmRight.setDirection(Servo.Direction.FORWARD);
        outtakeArmLeft.setDirection(Servo.Direction.REVERSE);
        outtakeClaw = new CachingServo(hardwareMap.get(Servo.class, "outtakeClaw"));

        // LOCALIZATION
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // X Pod Offset: 25.04879
        // Y Pod Offset: -45.26356
        odo.setOffsets(25.04879, -45.26356);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // INTAKE
        intakeClaw = new CachingServo(hardwareMap.get(Servo.class, "intakeClaw"));
        intakeArmLeft = new CachingServo(hardwareMap.get(Servo.class, "intakeArmLeft"));
        intakeArmRight = new CachingServo(hardwareMap.get(Servo.class, "intakeArmRight"));
        intakeArmRight.setDirection(REVERSE);
        intakeArmRight.setDirection(Servo.Direction.REVERSE);
        intakeCoaxial = (ServoImplEx) hardwareMap.get(Servo.class, "intakeCoaxial");
        intakeCoaxial.setDirection(FORWARD);
        turretClaw = new CachingServo(hardwareMap.get(Servo.class, "turretClaw"));

        // EXTENDO
        linkageServoLeft = new CachingServo(hardwareMap.get(Servo.class, "linkageServoLeft"));
        linkageServoRight = new CachingServo(hardwareMap.get(Servo.class, "linkageServoRight"));
        linkageServoLeft.setDirection(REVERSE);

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
        drivetrain.read();
        intake.read();
        outtake.read();
        robotPose = odo.getPosition();
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

    public void setPose(Pose2D pose) {
        odo.setPosition(pose);
        robotPose = pose;
    }

    public Pose2D getPose() {
        return robotPose;
    }
}
