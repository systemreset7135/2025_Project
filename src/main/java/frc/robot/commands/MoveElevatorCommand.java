package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_targetHeight;

    public MoveElevatorCommand(ElevatorSubsystem elevator, double targetHeight) {
        m_elevator = elevator;
        m_targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setTargetHeight(m_targetHeight);
    }

    @Override
    public void execute() {
        // PID control updates in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }
}