// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
 
public class autoST extends SubsystemBase {
 
  public SparkMax shooter = new SparkMax(ShooterConstants.kShooterPort, MotorType.kBrushed);
  public boolean shooting = false;
  public boolean haveShot = false;
 
  Timer timer = new Timer();
 
 
  public autoST() {}
 
 
  @Override
  public void periodic() {
  }
 
  @Override
  public void simulationPeriodic() {
  }
  public void shoot(String direction){
    if(direction == "in"){
      shooter.set(ShooterConstants.kIntakeSpeed);
    }

    else if(direction == "out"){
      shooter.set(-ShooterConstants.kIntakeSpeed);
    }
    else{
      shooter.set(0);
    }
  }
}
 