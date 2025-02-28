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
    private final PS4Controller joystick2;

    public MEC(ElevatorSubsystem elevatorSubsystem, PS4Controller joystick, PS4Controller joystick2) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick = joystick;
        this.joystick2 = joystick2;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        // Read trigger values (range: 0 to 1)
        double rightTrigger = -MathUtil.applyDeadband(joystick.getRawAxis(3), OIConstants.kDriveDeadband) + 1;
        double leftTrigger = -MathUtil.applyDeadband(joystick.getRawAxis(4), OIConstants.kDriveDeadband) + 1; 

        // Calculate motor speed
        double motorSpeed = rightTrigger - leftTrigger ;


        double rightTrigger2 = -MathUtil.applyDeadband(joystick2.getRawAxis(4), OIConstants.kDriveDeadband) + 1;
        double leftTrigger2 = -MathUtil.applyDeadband(joystick2.getRawAxis(3), OIConstants.kDriveDeadband) + 1; 

        // Calculate motor speed
        double motorSpeed2 = rightTrigger2 - leftTrigger2 ;

        // Set the motor speed
        elevatorSubsystem.setMotorSpeed((motorSpeed2 - motorSpeed) * ElevatorConstants.kElevatorManualSpeed);
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