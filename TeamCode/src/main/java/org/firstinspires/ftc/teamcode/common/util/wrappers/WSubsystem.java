package org.firstinspires.ftc.teamcode.common.util.wrappers;

import androidx.annotation.NonNull;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.mercurial.subsystems.Subsystem;

public abstract class WSubsystem implements Subsystem {
    @NonNull
    public abstract Dependency<?> getDependency();
    public abstract void setDependency(@NonNull Dependency<?> dependency);
    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
