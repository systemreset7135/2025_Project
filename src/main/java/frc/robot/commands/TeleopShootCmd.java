// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;
 
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
 
/** An example command that uses an example subsystem. */
public class TeleopShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final String direction;
 
  public TeleopShootCmd(ShooterSubsystem shooterSubsystem, String direction) {
    this.direction = direction;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }
 
  @Override
  public void initialize() {
    System.out.println("ShootingCmd started!");
  }
 
  @Override
  public void execute() {
    shooterSubsystem.shoot(direction);
  }
 
  @Override
  public void end(boolean interrupted) {
    System.out.println("ShootingCmd ended!");
  }
 
  @Override
  public boolean isFinished() {
    return true;
  }
}
 