package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ElevatorSubsystem extends SubsystemBase {
    private final PIDController m_pidController;
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
   
    private double targetHeight = 0.0; //목표 높이 
    private Double lastPrintedHeight = null; //마지막으로 출력한 높이 

    public ElevatorSubsystem() {
        
        m_motor = new SparkMax(ElevatorConstants.ElevatorCanId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        // Initialize PID controller
        m_pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        m_pidController.setTolerance(ElevatorConstants.kTolerance);
        System.out.println("ElevatorSubsystem initialized.");
    }

    // 목표 높이 설정
    public void setTargetHeight(double height) {
        if (Math.abs(targetHeight - height) > 1e-3) { // 목표값이 변경될 때만 출력
            System.out.println("Elevator moving to new target: " + height + " inches");
            targetHeight = height;
            lastPrintedHeight = height;
        }
    }
    public double getHeight() {
        return m_encoder.getPosition();
    }

    // Stop the motor
    public void stop() {
        m_motor.set(0);
        System.out.println("Elevator stopped.");
    }

    // Check if the elevator is at the desired position
    public boolean atSetpoint() {
        return m_pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        // Update PID loop regularly
    }
}