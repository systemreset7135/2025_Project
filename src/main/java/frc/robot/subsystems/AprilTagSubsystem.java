package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public AprilTagSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        // 주기적으로 호출되어 데이터를 업데이트함
        double[] tidArray = limelightTable.getEntry("tid").getDoubleArray(new double[6]);
        double tx = limelightTable.getEntry("tx").getDouble(0);
        double ty = limelightTable.getEntry("ty").getDouble(0);
        double tv = limelightTable.getEntry("tv").getDouble(0);
        double tlong = limelightTable.getEntry("tlong").getDouble(0);
        SmartDashboard.putNumber("peter tid", limelightTable.getEntry("tid").getDouble(6));
        // 데이터를 SmartDashboard에 업데이트
        SmartDashboard.putString("tid", arrayToString(tidArray));
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("tlong", tlong);

        // 인식 여부를 SmartDashboard에 표시
        if (tv != 0) {
            SmartDashboard.putString("AprilTag", "AprilTag Detected!");
            
        } else {
            SmartDashboard.putString("AprilTag", "No AprilTag Detected");
        }

    }
    public boolean isTagDetected() {
        double tv = limelightTable.getEntry("tv").getDouble(0);
        return tv != 0;  // Returns true if an AprilTag is detected, otherwise false
    }
    public double getTagDistance() {
        double ty = limelightTable.getEntry("ty").getDouble(0);
        return calculateDistance(ty);
    }
    public double getTagHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0);
    }
    private double calculateDistance(double ty) {
        double targetHeight = 1;
        double limelightHeight = 0.15;
        double limelightAngle = 18;

        double angleToTarget = limelightAngle + ty;
        return (targetHeight - limelightHeight) / Math.tan(Math.toRadians(angleToTarget));
    }

    // 배열을 문자열로 변환하는 도우미 메서드
    private String arrayToString(double[] array) {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < array.length; i++) {
            sb.append(array[i]);
            if (i < array.length - 1) {
                sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }
}
