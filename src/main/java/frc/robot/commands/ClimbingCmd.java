// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;
 
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
 
/** An example command that uses an example subsystem. */
public class ClimbingCmd extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final String direction;
    public ClimbingCmd(ClimberSubsystem climberSubsystem, String direction) {
    this.climberSubsystem = climberSubsystem;
    this.direction = direction;
    addRequirements(climberSubsystem);
    }
 
    @Override
    public void initialize() {
        System.out.println("ClimbCmd started!");
    }
 
    @Override
    public void execute() {
        climberSubsystem.climb(direction);
    }
 
    @Override
    public void end(boolean interrupted) {
        System.out.println("ClimbCmd ended!");
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}
 