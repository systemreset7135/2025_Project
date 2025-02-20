package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AI extends SubsystemBase {
    private final NetworkTable table;

    public AI() {
        // 예: Limelight 3의 AI 결과가 "LL3AI" 테이블에 올라온다고 가정
        table = NetworkTableInstance.getDefault().getTable("LL3AI");
    }

    // offsetX: 화면 중앙 기준 -0.5 (왼쪽) ~ +0.5 (오른쪽)
    public double getOffsetX() {
        return table.getEntry("offsetX").getDouble(0.0);
    }

    // offsetY: 화면 중앙 기준 -0.5 (위) ~ +0.5 (아래)
    public double getOffsetY() {
        return table.getEntry("offsetY").getDouble(0.0);
    }

    // 장애물 인식 신뢰도 (0~1)
    public double getConfidence() {
        return table.getEntry("confidence").getDouble(0.0);
    }

    // 간단히 임계값(예: 0.5 이상이면 장애물로 판단)
    public boolean isObstacleDetected() {
        return getConfidence() > 0.5;
    }
}
