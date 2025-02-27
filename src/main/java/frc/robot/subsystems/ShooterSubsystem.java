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
 
public class ShooterSubsystem extends SubsystemBase {
 
  SparkMax shooter = new SparkMax(ShooterConstants.kShooterPort, MotorType.kBrushed);
  public boolean shooting = false;
  public boolean haveShot = false;
 
  Timer timer = new Timer();
 
 
  public ShooterSubsystem() {}
 
 
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
    // if((direction == "out"  && shooting == false) || shooting == true){
    //   timer.start();
    //   shooting = true;
    //   SmartDashboard.putNumber("time dddd", timer.get());
    //   frontRightShoot.set(ControlMode.PercentOutput, -ShooterConstants.kShootSpeed);
    //   frontLeftShoot.set(ControlMode.PercentOutput, ShooterConstants.kShootSpeed);
    //   SmartDashboard.putNumber("frontRight", frontRightShoot.getMotorOutputPercent());
    //   SmartDashboard.putNumber("frontLeft", frontLeftShoot.getMotorOutputVoltage());
    //   if(timer.get() > ShooterConstants.kShootChargeTime && timer.get() < ShooterConstants.kShootChargeTime + 1.5){
    //     rearRightShoot.set(ControlMode.PercentOutput, -ShooterConstants.kShootSpeed);
    //     rearLeftShoot.set(ControlMode.PercentOutput, ShooterConstants.kShootSpeed);
    //     SmartDashboard.putNumber("rearRight", rearRightShoot.getMotorOutputPercent());
    //     SmartDashboard.putNumber("rearLeft", rearLeftShoot.getMotorOutputPercent());
       
    //   }
    //   if(timer.get() > ShooterConstants.kShootChargeTime + 1.5){
    //     shooting = false;
    //     timer.reset();
    //     timer.stop();
    //   }
    // }
    // else if((direction == "auto"  && shooting == false) || shooting == true){
    //   if(haveShot){
    //     return;
    //   }
    //   timer.start();
    //   shooting = true;
    //   SmartDashboard.putNumber("time dddd", timer.get());
    //   frontRightShoot.set(ControlMode.PercentOutput, -ShooterConstants.kShootSpeed);
    //   frontLeftShoot.set(ControlMode.PercentOutput, ShooterConstants.kShootSpeed);
    //   SmartDashboard.putNumber("frontRight", frontRightShoot.getMotorOutputPercent());
    //   SmartDashboard.putNumber("frontLeft", frontLeftShoot.getMotorOutputVoltage());
    //   if(timer.get() > ShooterConstants.kShootChargeTime && timer.get() < ShooterConstants.kShootChargeTime + 1.5){
    //     rearRightShoot.set(ControlMode.PercentOutput, -ShooterConstants.kShootSpeed);
    //     rearLeftShoot.set(ControlMode.PercentOutput, ShooterConstants.kShootSpeed);
    //     SmartDashboard.putNumber("rearRight", rearRightShoot.getMotorOutputPercent());
    //     SmartDashboard.putNumber("rearLeft", rearLeftShoot.getMotorOutputPercent());
       
    //   }
    //   if(timer.get() > ShooterConstants.kShootChargeTime + 1.5){
    //     shooting = false;
    //     timer.reset();
    //     timer.stop();
    //     haveShot = true;
    //   }
    // }
    else if(direction == "out"){
      shooter.set(-ShooterConstants.kIntakeSpeed);
    }
    else{
      shooter.set(0);
    }
  }
}
 