// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry ts;

    public Limelight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        ts = table.getEntry("ts");
    }

    public NetworkTable getTable() {
        return table;
    }

    public double getTargetX() {
        return tx.getDouble(0.0);
    }

    public double getTargetY() {
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public double getTargetSkew() {
        return ts.getDouble(0.0);
    }

    public boolean hasTarget() {
        return tv.getDouble(0.0) != 0.0;
    }

    public void setPipelineIndex(int index) {
        table.getEntry("pipeline").setDouble(index);
    }
}
