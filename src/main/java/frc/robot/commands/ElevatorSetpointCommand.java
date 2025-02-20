package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    private int currentIndex;
    private boolean setpointChanged = false;

    public ElevatorSetpointCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
        currentIndex = 0;
    }

    @Override
    public void execute() {
        if (setpointChanged) {
            elevator.setSetpoint(ElevatorConstants.kSetpoints[currentIndex]);
            setpointChanged = false;
        }
    }

    public void buttonPressed() {
        int newIndex = (currentIndex + 1) % ElevatorConstants.kSetpoints.length;
        if (ElevatorConstants.kSetpoints[newIndex] != ElevatorConstants.kSetpoints[currentIndex]) {
            currentIndex = newIndex;
            setpointChanged = true;
            System.out.println("[ElevatorSetpointCommand] New setpoint index: " + currentIndex);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // 이 명령은 상시 동작
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[ElevatorSetpointCommand] Command ended. Interrupted: " + interrupted);
    }
}