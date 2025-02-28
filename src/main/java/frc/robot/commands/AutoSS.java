package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.autoST;


public class AutoSS extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double startTime;

    public AutoSS(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        shooterSubsystem.shooter.set(-0.3); 
    }

    @Override
    public void execute() {
       
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shoot("stop");  
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        return elapsedTime >= 2;  
    }
}