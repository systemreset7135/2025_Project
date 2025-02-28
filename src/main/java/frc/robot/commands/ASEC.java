package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ASEC extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double targetSetpoint;
    private final double tolerance = Constants.ElevatorConstants.kElevatorSetpointTolerance; // Tolerance in inches
    private double START_TIME;

    public ASEC(ElevatorSubsystem elevatorSubsystem, double targetSetpoint) {
        this.m_elevator = elevatorSubsystem;
        this.targetSetpoint = targetSetpoint;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        START_TIME = Timer.getFPGATimestamp();
    
    }

    @Override
    public void execute() {
        double currentPosition = m_elevator.getEncoderDistance();
        double error = targetSetpoint - currentPosition;
        double speed = error > 0 ? ElevatorConstants.kElevatorSetpointSpeed : -ElevatorConstants.kElevatorSetpointSpeed;
        m_elevator.setMotorSpeed(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        double elapedTime = Timer.getFPGATimestamp() - START_TIME;
        //double currentPosition = m_elevator.getEncoderDistance();
        //boolean onTarget = Math.abs(currentPosition - targetSetpoint) < tolerance; 
  
        // return onTarget;

        return elapedTime >= 3.0;

    }

    public boolean isAttarget(){
        double currentPosition = m_elevator.getEncoderDistance();
        return Math.abs(currentPosition - targetSetpoint) < tolerance; 
    }
}