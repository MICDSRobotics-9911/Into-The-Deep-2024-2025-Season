package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends WSubsystem {

    // for intaking
    // armPos: 0.78
    // coaxialPos: 0.14
    // turretClaw: 0.3

    private final RobotHardware robot;

    private PivotState pivotState;

    public ClawState claw = ClawState.CLOSED;

    public enum CoaxialState {
        INTAKE,
        TRANSFER
    }

    public enum ArmState {
        INTAKE,
        TRANSFER1,
        TRANSFER2
    }

    public enum PivotState {
        EXTEND,
        RETRACT
    }

    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();
        reset();
    }

    public void updateState(@NotNull ClawState state) {
        double clawPosition = getClawStatePosition(state);
        robot.intakeClaw.setPosition(clawPosition);
        this.claw = state;
    }


    public void updateState(@NotNull PivotState state) {
        double pivotPosition = getPivotStatePosition(state);
        robot.linkageServoLeft.setPosition(pivotPosition);
        robot.linkageServoRight.setPosition(pivotPosition);
        this.pivotState = state;
    }


    @Override
    public void periodic() {
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        updateState(PivotState.RETRACT);
    }

    public double getCoaxialStatePosition(CoaxialState state) {
        switch (state) {
            case INTAKE:
                return 0;
            case TRANSFER:
                return 0.9;
            default:
                return 1;
        }
    }

    private double getArmStatePosition(ArmState state) {
        switch (state) {
            case INTAKE:
                return 0.8;
            case TRANSFER1:
                return 0.7;
            case TRANSFER2:
                return 0.93;
            default:
                return 1;
        }
    }

    private double getPivotStatePosition(PivotState state) {
        switch (state) {
            case RETRACT:
                return 0;
            case EXTEND:
                return 0.32;
            default:
                return 0;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return 0.9;
            case CLOSED:
                return 0.75;
            default:
                return 0.9;
        }
    }

    public ClawState getClawState() {
        return robot.intake.claw == ClawState.CLOSED
                ? ClawState.CLOSED : ClawState.OPEN;
    }
    public PivotState getPivotState() {
        return robot.intake.pivotState == PivotState.RETRACT
                ? PivotState.RETRACT : PivotState.EXTEND;
    }
}
