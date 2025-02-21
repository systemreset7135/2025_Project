package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor1; // CAN ID 23
    private final SparkMax elevatorMotor2; // CAN ID 24
    private final Encoder encoder; // 외부 엔코더
    private PIDController pidController;
    private double setpoint;
    private long lastPidUpdate = 0;
    private static final long PID_UPDATE_INTERVAL_MS = 50;

    public ElevatorSubsystem() {
        // 모터 초기화
        elevatorMotor1 = new SparkMax(ElevatorConstants.Elevator1CanId, MotorType.kBrushed); // 모터 타입 확인 필요
        elevatorMotor2 = new SparkMax(ElevatorConstants.Elevator2CanId, MotorType.kBrushed);
        
        // 엔코더 초기화 (디지털 입력 포트 예시: 0, 1)
        encoder = new Encoder(0, 1); // 실제 포트 번호로 수정
        encoder.setDistancePerPulse(1.0 / 360.0); // 예: 360 펄스/회전 = 1 인치 (실제 값으로 조정)

        // PID 컨트롤러 초기화
        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        pidController.setTolerance(ElevatorConstants.kTolerance);
        
        // 초기 세트포인트 설정
        setpoint = ElevatorConstants.kSetpoints[0];
    }

    public void setSetpoint(double height) {
        setpoint = Math.max(ElevatorConstants.MIN_HEIGHT, Math.min(height, ElevatorConstants.MAX_HEIGHT));
    }
    
    public double getSetpoint() {
        return setpoint;
    }

    public double getCurrentHeight() {
        return encoder.getDistance(); // 엔코더에서 인치 단위로 위치 반환
    }
    
    private void checkLimits() {
        double currentHeight = getCurrentHeight();
        if (currentHeight < ElevatorConstants.MIN_HEIGHT) {
            setSetpoint(ElevatorConstants.MIN_HEIGHT);
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
            System.out.println("[ElevatorSubsystem] Limit reached: Below MIN_HEIGHT");
        } else if (currentHeight > ElevatorConstants.MAX_HEIGHT) {
            setSetpoint(ElevatorConstants.MAX_HEIGHT);
            elevatorMotor1.set(0);
            elevatorMotor2.set(0);
            System.out.println("[ElevatorSubsystem] Limit reached: Above MAX_HEIGHT");
        }
    }

    @Override
    public void periodic() {
        long now = System.currentTimeMillis();
        if (now - lastPidUpdate >= PID_UPDATE_INTERVAL_MS) {
            lastPidUpdate = now;
            double currentHeight = getCurrentHeight();
            double output = MathUtil.clamp(pidController.calculate(currentHeight, setpoint), -1.0, 1.0);

            // 두 모터에 동일한 출력 적용
            elevatorMotor1.set(output);
            elevatorMotor2.set(output);
            
            // 안전성 체크
            checkLimits();
        }
        
        // SmartDashboard 업데이트
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator Error", setpoint - getCurrentHeight());
        SmartDashboard.putNumber("Elevator PID Output", elevatorMotor1.get());

        SmartDashboard.putNumber("Elevator P", pidController.getP());
        SmartDashboard.putNumber("Elevator I", pidController.getI());
        SmartDashboard.putNumber("Elevator D", pidController.getD());

        pidController.setP(SmartDashboard.getNumber("Elevator P", pidController.getP()));
        pidController.setI(SmartDashboard.getNumber("Elevator I", pidController.getI()));
        pidController.setD(SmartDashboard.getNumber("Elevator D", pidController.getD()));
    }

    public void setPID(double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }
    
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
    
    public void stop() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }
}