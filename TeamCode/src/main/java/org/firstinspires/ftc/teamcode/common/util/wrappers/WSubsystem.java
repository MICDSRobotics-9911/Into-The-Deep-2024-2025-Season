package org.firstinspires.ftc.teamcode.common.util.wrappers;

import androidx.annotation.NonNull;

public abstract class WSubsystem {
    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
