package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMortor1; 
    private final SparkMax m_elevatorMortor2;
    private final Encoder m_encoder;
    private final PIDController pidController;

    public ElevatorSubsystem() {
        m_elevatorMortor1 = new SparkMax(ElevatorConstants.kElevator1CanId, MotorType.kBrushed);
        m_elevatorMortor2 = new SparkMax(ElevatorConstants.kElevator2CanId, MotorType.kBrushed);
        m_encoder = new Encoder(ElevatorConstants.kEncoderDIOPortA, ElevatorConstants.kEncoderDIOPortB);
        m_encoder.setDistancePerPulse(ElevatorConstants.kEncoderDistancePerPulse);
        resetEncoder();
       

        pidController = new PIDController(0.1, 0.002, 0.0);
        
    }

    

    public void setMotorSpeed(double speed) {
        m_elevatorMortor1.set(speed);
        m_elevatorMortor2.set(speed);
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }
    
    public void resetEncoder() {
        m_encoder.reset();
    }
   
    
    @Override 
    public void periodic (){
        SmartDashboard.putData(m_encoder);
    }
}