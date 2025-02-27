package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class MEC extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final PS4Controller joystick;

    public MEC(ElevatorSubsystem elevatorSubsystem, PS4Controller joystick) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick = joystick;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        // Read trigger values (range: 0 to 1)
        double rightTrigger = -MathUtil.applyDeadband(joystick.getRawAxis(3), OIConstants.kDriveDeadband) + 1; // Right trigger (axis 3)
        double leftTrigger = -MathUtil.applyDeadband(joystick.getRawAxis(4), OIConstants.kDriveDeadband) + 1;  // Left trigger (axis 2)

        // Calculate motor speed
        double motorSpeed = rightTrigger - leftTrigger ;

        // Set the motor speed
        elevatorSubsystem.setMotorSpeed(motorSpeed * ElevatorConstants.kElevatorManualSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        elevatorSubsystem.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own; it runs continuously
        return false;
    }
}