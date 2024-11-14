package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.teamcode.common.gamespecific.ClawSide;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends WSubsystem {

    private final RobotHardware robot;

    private PivotState pivotState;

    public ClawState claw = ClawState.CLOSED;

    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        OPEN,
        AUTO
    }

    public enum PivotState {
        FLAT,
        STORED,
        SCORING
    }

    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();

        updateState(ClawState.CLOSED, ClawSide.BOTH);
    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);
        if (side == ClawSide.BOTH) {
            this.claw = state;
        }
    }

    public void updateState(@NotNull PivotState state) {
        this.pivotState = state;
    }


    @Override
    public void periodic() {
        if (pivotState == PivotState.SCORING) {
            // Scoring code
        } else if (pivotState == PivotState.FLAT) {
            // flat
        } else if (pivotState == PivotState.STORED) {
            // set target position
        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        updateState(PivotState.STORED);
    }

    private double getClawStatePosition(ClawState state, ClawSide side) {
        // TODO: Tune this
        return 0.5;
    }

    public ClawState getClawState(ClawSide side) {
        return robot.intake.claw == IntakeSubsystem.ClawState.CLOSED
                ? ClawState.CLOSED : ClawState.OPEN;
    }
}
