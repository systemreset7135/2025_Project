package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    
    private PIDController pidController;
    
    private double setpoint;
    private long lastPidUpdate = 0;
    private static final long PID_UPDATE_INTERVAL_MS = 50; // PID 조절 주기

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(ElevatorConstants.ElevatorCanId, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();
        
        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        pidController.setTolerance(ElevatorConstants.kTolerance);
        
        setpoint = ElevatorConstants.kSetpoints[0];
    }

    public void setSetpoint(double height) {
        setpoint = Math.max(ElevatorConstants.MIN_HEIGHT, Math.min(height, ElevatorConstants.MAX_HEIGHT));
    }
    
    public double getSetpoint() {
        return setpoint;
    }

    public double getCurrentHeight() {
        return elevatorEncoder.getPosition();
    }
    
    private void checkLimits() {
        double currentHeight = getCurrentHeight();
        if (currentHeight < ElevatorConstants.MIN_HEIGHT) {
            setSetpoint(ElevatorConstants.MIN_HEIGHT);
            elevatorMotor.set(0); // Stop motor at minimum height
            System.out.println("[ElevatorSubsystem] Limit reached: Below MIN_HEIGHT");
        } else if (currentHeight > ElevatorConstants.MAX_HEIGHT) {
            setSetpoint(ElevatorConstants.MAX_HEIGHT);
            elevatorMotor.set(0); // Stop motor at maximum height
            System.out.println("[ElevatorSubsystem] Limit reached: Above MAX_HEIGHT");
        }
    }

    @Override
    public void periodic() {
        long now = System.currentTimeMillis();
        if (now - lastPidUpdate >= PID_UPDATE_INTERVAL_MS) {
            lastPidUpdate = now;
            double currentHeight = getCurrentHeight();
            double error = setpoint - currentHeight;
            double output = MathUtil.clamp(pidController.calculate(currentHeight, setpoint), -1.0, 1.0); // 출력 제한

            elevatorMotor.set(output);
            
            // 안전성 체크
            checkLimits();
        }
        
        // SmartDashboard 업데이트는 여전히 주기적으로
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putNumber("Elevator Current Height", getCurrentHeight());
        SmartDashboard.putNumber("Elevator Error", setpoint - getCurrentHeight());
        SmartDashboard.putNumber("Elevator PID Output", elevatorMotor.get());

        // PID 값을 SmartDashboard에 업데이트
        SmartDashboard.putNumber("Elevator P", pidController.getP());
        SmartDashboard.putNumber("Elevator I", pidController.getI());
        SmartDashboard.putNumber("Elevator D", pidController.getD());

        // SmartDashboard에서 PID 값을 실시간으로 튜닝
        pidController.setP(SmartDashboard.getNumber("Elevator P", pidController.getP()));
        pidController.setI(SmartDashboard.getNumber("Elevator I", pidController.getI()));
        pidController.setD(SmartDashboard.getNumber("Elevator D", pidController.getD()));
    }

    // PID 값을 직접 설정하는 메서드
    public void setPID(double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }
    
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
    
    public void stop() {
        elevatorMotor.set(0);
    }
}