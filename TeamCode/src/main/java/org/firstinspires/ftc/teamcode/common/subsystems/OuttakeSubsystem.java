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

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public boolean usePIDF = true;

    public ClawState claw = ClawState.OPEN;
    public SlideState slide = SlideState.RESET;
    private double pivotTarget = 0;
    private int target = 0;

    public OuttakeSubsystem() {
        updateState(ClawState.OPEN);
        updateState(SlideState.RESET);
        updateState(PivotState.RESET);
        pid = 0;
        controller = new PIDFController(p, i, d, f);
        controller.reset();
    }
    @Override
    public void periodic() {
        int motorPos = 0;
        if (usePIDF)
            motorPos = robot.extension.getCurrentPosition();

        pid = controller.calculate(
                motorPos, target);
        if (usePIDF) {
            robot.extension.setPower(pid);
        }
        pivotTarget = robot.outtakeArm.getPosition();
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
        robot.outtakeArm.setPosition(pivotPosition);
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        controller.reset();
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return 0.7;
            case CLOSED:
                return 0;
            default:
                return 0;
        }
    }

    private int getSlideStatePosition(SlideState state) {
        switch (state) {
            case HIGH_BASKET:
                return 1000;
            case LOW_BASKET:
                return 500;
            case SPECIMEN_INTAKE:
                return 100;
            case SPECIMEN_OUTTAKE:
                return 300;
            case RESET:
            default:
                return 0;
        }
    }

    private double getPivotStatePosition(PivotState state) {
        switch (state) {
            case SCORING:
                return 0;
            case INTAKING:
                return 0.8;
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

    public SlideState getSlideState() {
        return robot.outtake.slide;
    }

    public void setTargetPosition(int position) {
        this.target = position;
    }

    public double getPID() {
        return pid;
    }

}
