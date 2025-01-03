package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;

public class IntakeSubsystem extends WSubsystem {

    private final RobotHardware robot;

    private PivotState pivotState;

    public ClawState claw = ClawState.CLOSED;

    public enum PivotState {
        EXTEND,
        RETRACT
    }

    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();
        //updateState(ClawState.OPEN);
    }

    public void updateState(@NotNull ClawState state) {
        double clawPosition = getClawStatePosition(state);
        robot.intakeClaw.setPosition(clawPosition);
        this.claw = state;
    }

    public void updateState(@NotNull PivotState state) {
        double pivotPosition = getPivotStatePosition(state);
        robot.linkageServo.setPosition(pivotPosition);
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
        //updateState(PivotState.RETRACT);
    }

    private double getPivotStatePosition(PivotState state) {
        switch (state) {
            case RETRACT:
                return 0;
            case EXTEND:
                return 1;
            default:
                return 0;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return 1;
            case CLOSED:
                return 0;
            default:
                return 0;
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
