package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class OuttakeSubsystem extends WSubsystem {

    public enum SlideState {
        HIGH_BASKET,
        LOW_BASKET,
        SPECIMEN_INTAKE,
        SPECIMEN_OUTTAKE,
        INCREMENT,
        DECREMENT,
        SPECIMEN_SCORING,
        RESET
    }

    public enum PivotState {
        SCORING,
        INTAKING,
        INCREMENT,
        DECREMENT,
        RESET,
    }

    private final RobotHardware robot = RobotHardware.getInstance();
    private PIDFController controller;
    private double pid;

    public static double p = 0.033;
    public static double d = 0;
    public boolean usePIDF = true;

    public ClawState claw = ClawState.OPEN;
    public SlideState slide = SlideState.RESET;
    public PivotState arm = PivotState.RESET;
    private double pivotTarget = 0;
    public int slideTarget = 0;
    private int motorTicks = 0;
    public static int specimenOuttake = 1500;
    public static int specimenIntake = 500;
    public static int tolerance = 5;

    public OuttakeSubsystem() {
        updateState(ClawState.OPEN);
        updateState(SlideState.RESET);
        updateState(PivotState.RESET);
        pid = 0;
        controller = new PIDFController(p, 0, d, 0);
        controller.reset();
    }
    @Override
    public void periodic() {
        pivotTarget = robot.outtakeArmRight.getPosition();
    }

    public void updateState(@NotNull ClawState state) {
        double clawPosition = getClawStatePosition(state);
        robot.outtakeClaw.setPosition(clawPosition);
        this.claw = state;
    }

    public void updateState(@NotNull SlideState state) {
        int slidePosition = getSlideStatePosition(state);
        setTargetPosition(slidePosition);
        this.slide = state;
    }

    public void updateState(@NotNull PivotState state) {
        double pivotPosition = getPivotStatePosition(state);
        robot.outtakeArmLeft.setPosition(pivotPosition);
        robot.outtakeArmRight.setPosition(pivotPosition);
        this.arm = state;
    }

    @Override
    public void read() {
        motorTicks = robot.extensionRight.getCurrentPosition();
    }

    @Override
    public void write() {
        pid = controller.calculate(motorTicks, slideTarget);
        if (usePIDF && Math.abs(slideTarget - motorTicks) > tolerance) {
            robot.extensionRight.setPower(pid);
        } else {
            robot.extensionRight.setPower(0);
        }
    }

    @Override
    public void reset() {
        controller.reset();
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return 0.9;
            case CLOSED:
                return 0.3;
            default:
                return 0;
        }
    }

    private int getSlideStatePosition(SlideState state) {
        // 3440 is the top
        switch (state) {
            case HIGH_BASKET:
                return 3000;
            case LOW_BASKET:
                return 500;
            case SPECIMEN_OUTTAKE:
                return specimenOuttake;
            case INCREMENT:
                return slideTarget + 100;
            case DECREMENT:
                return slideTarget - 100;
            case SPECIMEN_INTAKE:
                return specimenIntake;
            case SPECIMEN_SCORING:
                return 1500;
            case RESET:
            default:
                return 0;
        }
    }

    private double getPivotStatePosition(PivotState state) {
        switch (state) {
            case SCORING:
                return 1;
            case INTAKING:
                return 0;
            case INCREMENT:
                return pivotTarget + 0.1;
            case DECREMENT:
                return pivotTarget - 0.1;
            case RESET:
            default:
                return 0;
        }
    }

    public ClawState getClawState() {
        return robot.outtake.claw == ClawState.CLOSED
                ? ClawState.CLOSED : ClawState.OPEN;
    }

    public PivotState getArmState() {
        return robot.outtake.arm;
    }

    public SlideState getSlideState() {
        return robot.outtake.slide;
    }

    public void setTargetPosition(int position) {
        this.slideTarget = position;
    }

    public double getPID() {
        return pid;
    }

}
