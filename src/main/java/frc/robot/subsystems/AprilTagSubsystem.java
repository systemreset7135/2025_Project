package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private final NetworkTableInstance table = NetworkTableInstance.getDefault();

    public boolean hasValidTarget() {
        return table.getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetX() {
        return table.getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return table.getTable("limelight").getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return table.getTable("limelight").getEntry("ta").getDouble(0);
    }
}
