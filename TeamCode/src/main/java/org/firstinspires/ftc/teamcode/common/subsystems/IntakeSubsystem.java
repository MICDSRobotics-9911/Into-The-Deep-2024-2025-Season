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
    public ArmState arm = ArmState.RESET;
    public CoaxialState coaxial = CoaxialState.INTAKE;
    public TurretState turret = TurretState.INTAKE;

    private double turretPos = 0.3;
    private double armPos = 0.8;
    private double linkagePos = 0;

    public enum CoaxialState {
        INTAKE,
        TRANSFER,
    }

    public enum TurretState {
        INTAKE,
        TRANSFER,
        INCREMENT,
        DECREMENT
    }

    public enum ArmState {
        INTAKE,
        TRANSFER1,
        TRANSFER2,
        RESET
    }

    public enum PivotState {
        EXTEND,
        RETRACT,
        INCREMENT,
        DECREMENT
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

    public void updateState(@NotNull ArmState state) {
        double position = getArmStatePosition(state);
        robot.intakeArmLeft.setPosition(position);
        robot.intakeArmRight.setPosition(position);
        this.arm = state;
    }

    public void updateState(@NotNull CoaxialState state) {
        double coaxialPos = getCoaxialStatePosition(state);
        robot.intakeCoaxial.setPosition(coaxialPos);
        coaxial = state;
    }

    public void updateState(@NotNull TurretState state) {
        double turretPos = getTurretStatePosition(state);
        robot.turretClaw.setPosition(turretPos);
        turret = state;
    }


    @Override
    public void periodic() {
        armPos = robot.intakeArmLeft.getPosition();
        turretPos = robot.turretClaw.getPosition();
        linkagePos = robot.linkageServoLeft.getPosition();
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {
        updateState(ArmState.RESET);
        updateState(PivotState.RETRACT);
        updateState(ClawState.OPEN);
        updateState(CoaxialState.INTAKE);
    }

    private double getCoaxialStatePosition(CoaxialState state) {
        switch (state) {
            case INTAKE:
                return 0.4;
            case TRANSFER:
                return 0.9;
            default:
                return 1;
        }
    }

    private double getArmStatePosition(ArmState state) {
        switch (state) {
            case INTAKE:
                return 0.78;
            case TRANSFER1:
                return 0.7;
            case TRANSFER2:
                return 0.93;
            case RESET:
            default:
                return 0.9;
        }
    }

    private double getPivotStatePosition(PivotState state) {
        switch (state) {
            case RETRACT:
                return 0;
            case EXTEND:
                return 0.32;
            case DECREMENT:
                return linkagePos - 0.05;
            case INCREMENT:
                if (linkagePos + 0.05 >= 0.32)
                    return 0.32;
                else
                    return linkagePos + 0.05;
            default:
                return 0;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return 0.9;
            case CLOSED:
                return 0.7;
            default:
                return 0.9;
        }
    }

    private double getTurretStatePosition(TurretState state) {
        switch (state) {
            case INTAKE:
                return 0.3;
            case TRANSFER:
                return 0.3;
            case INCREMENT:
                return turretPos + 0.1;
            case DECREMENT:
                return turretPos - 0.1;
            default:
                return 0.3;
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
