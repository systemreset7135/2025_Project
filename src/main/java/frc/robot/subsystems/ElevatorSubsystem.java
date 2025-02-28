package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotor1;
    private final SparkMax m_elevatorMotor2;
    private final Encoder m_encoder;
    private final PIDController pidController;
    private double targetSetpoint = 0.0; // 동적으로 변경 가능
    private boolean usePID = false;

    public ElevatorSubsystem() { // targetSetpoint 제거
        m_elevatorMotor1 = new SparkMax(ElevatorConstants.kElevator1CanId, MotorType.kBrushed);
        m_elevatorMotor2 = new SparkMax(ElevatorConstants.kElevator2CanId, MotorType.kBrushed);
        m_encoder = new Encoder(ElevatorConstants.kEncoderDIOPortA, ElevatorConstants.kEncoderDIOPortB);
        m_encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        resetEncoder();
        pidController = new PIDController(0.1, 0.002, 0.0);
    }

    @Override
    public void periodic() {
        if (usePID) {
            double output = pidController.calculate(getEncoderDistance(), targetSetpoint);
            output = Math.max(-0.4, Math.min(0.4, output));
            setMotorSpeed(output);
        }
        SmartDashboard.putNumber("Elevator Position", getEncoderDistance());
    }

    public void setMotorSpeed(double speed) {
        m_elevatorMotor1.set(speed);
        m_elevatorMotor2.set(speed);
    }

    public void setSetpoint(double setpoint) {
        targetSetpoint = setpoint;
        usePID = true; // PID 활성화
    }

    public void stopPID() {
        usePID = false;
        setMotorSpeed(0.05); // 유지 전력 적용 (조정 필요)
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }
}
