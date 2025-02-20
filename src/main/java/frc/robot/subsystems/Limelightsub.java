package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelightsub 클래스는 Limelight 비전 시스템의 데이터를 처리하고 장애물 회피에 사용됩니다.
 * 이 클래스는 네트워크 테이블에서 데이터를 읽고, 필터링하여 정확한 장애물 감지를 제공합니다.
 */
public class Limelightsub extends SubsystemBase {
    private final NetworkTable table; // Limelight의 네트워크 테이블에 접근하기 위한 객체

    // 데이터 필터링을 위한 변수들
    private double filteredTX = 0, filteredTY = 0, filteredTA = 0; // 필터링된 tx, ty, ta 값 저장
    private int filterCount = 0; // 필터링에 사용된 샘플 수
    private static final int FILTER_SAMPLES = 5; // 필터링에 사용할 샘플 수

    public Limelightsub() {
        table = NetworkTableInstance.getDefault().getTable("limelight"); // Limelight 네트워크 테이블 초기화
    }

    /**
     * Limelight의 타겟 수평 오프셋 값을 반환
     * @return 타겟의 수평 오프셋 (도)
     */
    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }
    
    /**
     * Limelight의 타겟 수직 오프셋 값을 반환
     * @return 타겟의 수직 오프셋 (도)
     */
    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    
    /**
     * Limelight의 타겟 면적 값을 반환
     * @return 타겟의 면적 (%)
     */
    public double getTA() {
        return table.getEntry("ta").getDouble(0.0);
    }
    
    /**
     * Limelight가 유효한 타겟을 감지했는지 여부를 반환
     * @return 타겟이 유효하면 true, 아니면 false
     */
    public boolean hasValidTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    // 필터링된 값을 반환하는 메서드들
    /**
     * 필터링된 타겟의 수평 오프셋을 반환
     * @return 필터링된 tx 값
     */
    public double getFilteredTX() {
        updateFilteredValues();
        return filteredTX;
    }

    /**
     * 필터링된 타겟의 수직 오프셋을 반환
     * @return 필터링된 ty 값
     */
    public double getFilteredTY() {
        updateFilteredValues();
        return filteredTY;
    }

    /**
     * 필터링된 타겟의 면적을 반환
     * @return 필터링된 ta 값
     */
    public double getFilteredTA() {
        updateFilteredValues();
        return filteredTA;
    }

    // 데이터 필터링 메서드
    private void updateFilteredValues() {
        // 현재 값을 가져옴
        double newTX = getTX();
        double newTY = getTY();
        double newTA = getTA();
        
        // 이동 평균 필터 적용
        filteredTX = (filteredTX * filterCount + newTX) / (filterCount + 1);
        filteredTY = (filteredTY * filterCount + newTY) / (filterCount + 1);
        filteredTA = (filteredTA * filterCount + newTA) / (filterCount + 1);
        filterCount = Math.min(filterCount + 1, FILTER_SAMPLES); // 샘플 수를 최대 FILTER_SAMPLES로 제한
    }

    /**
     * 장애물이 감지되었는지 여부를 반환합니다. 
     * 타겟의 면적이 크고, 거리가 가까울 때 장애물로 판단합니다.
     * @return 장애물이 감지되면 true, 아니면 false
     */
    public boolean isObstacleDetected() {
        if (hasValidTarget()) {
            double ta = getFilteredTA();
            double ty = getFilteredTY();
            double distance = calculateDistance(ty);
            // 타겟 면적이 2.0 이상이고, 거리가 2미터 이내이면 장애물로 판단
            return ta > 2.0 && distance < 2.0; 
        }
        return false;
    }

    // 거리 계산 메서드
    public double calculateDistance(double ty) {
        // Limelight 카메라의 높이, 타겟의 높이, 타겟의 수직 오프셋을 이용한 거리 계산
        // 이 값은 실제 로봇과 환경에 맞게 조정해야 합니다.
        double cameraHeight = 0.5; // 카메라 높이 (미터)
        double targetHeight = 1.0; // 타겟 높이 (미터)
        double cameraAngle = Math.toRadians(20); // 카메라의 수평 각도 (라디안)
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(ty));
    }

    /**
     * 장애물의 상대적 수평 위치를 반환
     * @return 필터링된 tx 값
     */
    public double getOffsetX() {
        return getFilteredTX();
    }

    /**
     * 장애물의 상대적 수직 위치를 반환
     * @return 필터링된 ty 값
     */
    public double getOffsetY() {
        return getFilteredTY();
    }

    @Override
    public void periodic() {
        if (hasValidTarget()) {
            // 디버깅을 위해 현재 데이터를 출력
            System.out.printf("Limelight: TX=%.2f, TY=%.2f, TA=%.2f, Distance=%.2f\n", 
                              getFilteredTX(), getFilteredTY(), getFilteredTA(), calculateDistance(getFilteredTY()));
        }
    }
}