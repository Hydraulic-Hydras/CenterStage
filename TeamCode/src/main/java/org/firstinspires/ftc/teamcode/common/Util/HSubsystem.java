package org.firstinspires.ftc.teamcode.common.Util;

import com.arcrobotics.ftclib.command.SubsystemBase;
public abstract class HSubsystem extends SubsystemBase {

    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
