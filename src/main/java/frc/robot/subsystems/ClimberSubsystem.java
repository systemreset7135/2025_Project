// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
 
import frc.robot.Constants.ClimberConstants;
 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class ClimberSubsystem extends SubsystemBase {
 
  Spark leftActuator = new Spark(ClimberConstants.kLeftActuatorPort);
  Spark rightActuator = new Spark(ClimberConstants.kRightActuatorPort);
 
 
  Timer timer = new Timer();
  public ClimberSubsystem() {}
 
 
  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber timer", Timer.getFPGATimestamp());
  }
 
  @Override
  public void simulationPeriodic() {
  }
  public void climb(String direction){
    if(direction == "up"){
        System.out.println("UPPPP");
        leftActuator.set(-1);
        rightActuator.set(-1);
    }
    else if(direction == "down"){
        System.out.println("DOWNNNN");
        leftActuator.set(1);
        rightActuator.set(1);
    }
    else if(direction == "auto"){
        while(true){
            while(Timer.getMatchTime() > ClimberConstants.kStartAutoLiftTime && Timer.getMatchTime() < ClimberConstants.kAutoLiftDuration + ClimberConstants.kStartAutoLiftTime){
                leftActuator.set(-1);
                rightActuator.set(-1);
            }
            if(Timer.getMatchTime() > ClimberConstants.kAutoLiftDuration + ClimberConstants.kStartAutoLiftTime){
                break;
            }
        }
    }
    else{                                                                                                                                                                                                                                                                                                                                              
        leftActuator.set(0);
        rightActuator.set(0);
    }
  }
}
 
