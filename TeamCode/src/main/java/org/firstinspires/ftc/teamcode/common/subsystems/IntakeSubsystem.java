package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.ClawState;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;
@Config
public class IntakeSubsystem extends WSubsystem {

    // for intaking
    // armPos: 0.78
    // coaxialPos: 0.32

    // clawPos:

    // for transfer
    // armPos: 0.9
    // coaxialPos: 1
    // linkagePos: 0
    // turretPos: 0.48

    // for RESET

    private final RobotHardware robot;

    private PivotState pivotState;

    public ClawState claw = ClawState.CLOSED;
    public ArmState arm = ArmState.RESET;
    public CoaxialState coaxial = CoaxialState.INTAKE;
    public TurretState turret = TurretState.INTAKE;

    private double turretPos = 0.3;
    private double armPos = 0.8;
    private double linkagePos = 0;
    public static double RESET = 0.9;
    private double coaxialPos = 0;

    public enum CoaxialState {
        EXTEND,
        INTAKE,
        TRANSFER,
        INCREMENT,
        DECREMENT
    }

    public enum TurretState {
        INTAKE,
        TRANSFER,
        INCREMENT,
        DECREMENT
    }

    public enum ArmState {
        EXTEND,
        INTAKE,
        SUBMERSIBLE,
        TRANSFER,
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
        double clawPosition = MathUtils.clip(getClawStatePosition(state), 0, 0.24) ;
        robot.intakeClaw.setPosition(clawPosition);
        this.claw = state;
    }


    public void updateState(@NotNull PivotState state) {
        double pivotPosition = MathUtils.clip(getPivotStatePosition(state), 0, 0.32);
        robot.linkageServoLeft.setPosition(pivotPosition);
        robot.linkageServoRight.setPosition(pivotPosition);
        this.pivotState = state;
    }

    public void updateState(@NotNull ArmState state) {
        double position = MathUtils.clip(getArmStatePosition(state), 0.78, 0.9);
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
        coaxialPos = robot.intakeCoaxial.getPosition();
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
                return 0.32;
            case TRANSFER:
                return 1;
            case INCREMENT:
                return coaxialPos + 0.05;
            case DECREMENT:
                return coaxialPos - 0.05;
            default:
                return 1;
        }
    }

    private double getArmStatePosition(ArmState state) {
        switch (state) {
            case INTAKE:
                return 0.78;
            case SUBMERSIBLE:
                return 0.82;
            case TRANSFER:
                return 0.9;
            case RESET:
            default:
                return 0.85;
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
                return linkagePos + 0.05;
            default:
                return 0;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case CLOSED:
                return 0.0;
            case OPEN:
            default:
                return 0.24;
        }
    }

    private double getTurretStatePosition(TurretState state) {
        switch (state) {
            case INTAKE:
                return 0.48;
            case TRANSFER:
                return 0.48;
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
