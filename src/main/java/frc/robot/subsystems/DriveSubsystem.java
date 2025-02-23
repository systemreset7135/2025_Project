// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//전체 로봇 구동 시스템의 핵심 "엔진"과 같은 역할을 수행합니다.
package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DriveSubsystem extends SubsystemBase {
  // 다양한 방법으로 움지길수 있는 
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final GyroSubsystem m_gyro;
  private GenericEntry poseXEntry;
  private GenericEntry poseYEntry;
  private GenericEntry poseRotationEntry;
  private final Field2d fieldSim = new Field2d();

  SwerveDriveOdometry m_odometry;

  private final PIDController pidControllerX;
  private final PIDController pidControllerY;


  public DriveSubsystem(GyroSubsystem gyro) {
    this.m_gyro = gyro;
    SmartDashboard.putData("Field2D", fieldSim);
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

     var tab = Shuffleboard.getTab("Drive");
     poseXEntry = tab.add("Pose-X", 0.0).withPosition(0, 0).getEntry();
     poseYEntry = tab.add("Pose-Y", 0.0).withPosition(1, 0).getEntry();
     poseRotationEntry = tab.add("Pose-Rotation", 0.0).withPosition(2, 0).getEntry();
    
    
    
     m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

      pidControllerX = new PIDController(
            Constants.AutoConstants.kPXController,
            Constants.AutoConstants.kIXController,
            Constants.AutoConstants.kDXController
        );
        pidControllerY = new PIDController(
            Constants.AutoConstants.kPYController,
            Constants.AutoConstants.kIYController,
            Constants.AutoConstants.kDYController
        );
   }
 
   @Override
   public void periodic() {
      m_odometry.update(
        m_gyro.getRotation2d(),
         new SwerveModulePosition[] {
             m_frontLeft.getPosition(),
             m_frontRight.getPosition(),
             m_rearLeft.getPosition(),
             m_rearRight.getPosition()
         });
         m_frontLeft.debug();
         m_frontRight.debug();
         m_rearLeft.debug();
         m_rearRight.debug();

     Pose2d currentPose = m_odometry.getPoseMeters();
 
     SmartDashboard.putNumber("Pose X", currentPose.getX());
     SmartDashboard.putNumber("Pose Y", currentPose.getY());
     SmartDashboard.putNumber("Pose Rotation (deg)", currentPose.getRotation().getDegrees());
     SmartDashboard.putNumber("Current Speed", getCurrentSpeed());
     
     // 🔹 Shuffleboard에서 기존 Entry 값을 업데이트 (반복 등록 X)
     poseXEntry.setDouble(currentPose.getX());
     poseYEntry.setDouble(currentPose.getY());
     poseRotationEntry.setDouble(currentPose.getRotation().getDegrees());
         
     fieldSim.setRobotPose(m_odometry.getPoseMeters());

  
   
    }


  public Pose2d getPose() {
    return m_odometry.getPoseMeters();//현재 로봇의 위치(좌표와 방향)를 반환합니다 
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);


}
//Xspeed 앞뒤 
//Yspeed 좌우
//rot 회전
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
  double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
  double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
              m_gyro.getRotation2d())
          : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
}

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);  
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.zeroHeading();
}

public double getHeading() {
    return m_gyro.getHeading();
}

public double getTurnRate() {
    return m_gyro.getTurnRate();
}

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] moduleStates = {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
    return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
  }

  public double getCurrentSpeed() { 
    // 각 모듈의 속도를 가져와서 평균을 계산합니다.
    double totalSpeed = 0.0;
    for (MAXSwerveModule module : new MAXSwerveModule[]{m_frontLeft, m_frontRight, m_rearLeft, m_rearRight}) {
        SwerveModuleState state = module.getState();
        totalSpeed += Math.abs(state.speedMetersPerSecond);
    }
    return totalSpeed / 4.0; // 평균 속도를 계산
  }

public void driveWithPID(double targetX, double targetY, double currentXSpeed, double currentYSpeed) {
    double xOutput = pidControllerX.calculate(currentXSpeed, targetX);
    double yOutput = pidControllerY.calculate(currentYSpeed, targetY);
    drive(xOutput, yOutput, 0.0, true);
}
}
// 상황별 수정 방법:
        // 1. "확 멈춤" 문제 발생 시 (부드러운 감속 필요):
        //    - Constants.AutoConstants.kDXController와 kDYController를 0.01~0.05로 증가시켜 감속 완화.
        //    - 예: pidControllerX.setD(0.02); pidControllerY.setD(0.02);
        // 2. 속도 반응이 느릴 때:
        //    - Constants.AutoConstants.kPXController와 kPYController를 1.2~1.5로 증가시켜 반응성 향상.
        // 3. 작은 오차가 지속될 때:
        //    - Constants.AutoConstants.kIXController와 kIYController를 0.001~0.005로 설정해 오차 보정.
        // 4. 흔들림(오실레이션) 발생 시:
        //    - kPXController, kPYController를 0.8~0.9로 낮추고, kDXController, kDYController를 0.02로 증가.