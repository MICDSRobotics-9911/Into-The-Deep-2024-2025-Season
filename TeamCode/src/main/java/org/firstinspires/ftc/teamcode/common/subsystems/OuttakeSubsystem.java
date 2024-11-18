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

    private final RobotHardware robot = RobotHardware.getInstance();
    private PIDFController controller;
    private double pid;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public boolean usePIDF = true;

    public ClawState claw = ClawState.CLOSED;

    private int target = 0;

    public OuttakeSubsystem() {
        updateState(ClawState.OPEN);
        pid = 0;
        controller = new PIDFController(p, i, d, f);
    }
    @Override
    public void periodic() {
        pid = controller.calculate(
                (Double) robot.values.get(Sensors.SensorType.EXTENSION_ENCODER), target);
        if (usePIDF) {
            robot.extension.setPower(pid);
        }
    }

    public void updateState(@NotNull ClawState state) {
        double clawPosition = getClawStatePosition(state);
        robot.outtakeClaw.setPosition(clawPosition);
        this.claw = state;
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
                return 1;
            case CLOSED:
                return 0;
            default:
                return 0;
        }
    }

    public ClawState getClawState() {
        return robot.outtake.claw == ClawState.CLOSED
                ? ClawState.CLOSED : ClawState.OPEN;
    }

    public void setTargetPosition(int position) {
        this.target = position;
    }

    public double getPID() {
        return pid;
    }


}
