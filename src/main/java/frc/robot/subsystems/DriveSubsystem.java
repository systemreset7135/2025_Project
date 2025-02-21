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
  private final Pigeon2 m_gyro = new Pigeon2(0);

  // 로봇의 위치와 방향을 추적하는 시스템입니다.자이로 기반으로 현재 위치가 어디 있는지 정확히 추적
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  //자동차에 시스템에 gps시스템을 켰을때 "현재 위치를" 처음에 기록하는것 과 같습니다 
  private GenericEntry poseXEntry;
  private GenericEntry poseYEntry;
  private GenericEntry poseRotationEntry;
  private final Field2d fieldSim = new Field2d();
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field2D", fieldSim);
    // max를 사용한다고 표시 합니다 
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
     // 🔹 Shuffleboard 등록 (생성자에서 한 번만 실행)
     var tab = Shuffleboard.getTab("Drive");
     poseXEntry = tab.add("Pose-X", 0.0).withPosition(0, 0).getEntry();
     poseYEntry = tab.add("Pose-Y", 0.0).withPosition(1, 0).getEntry();
     poseRotationEntry = tab.add("Pose-Rotation", 0.0).withPosition(2, 0).getEntry();
   }
 
   @Override
   public void periodic() {
     // 로봇 위치 업데이트
     m_odometry.update(
         Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
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
 
     // 현재 로봇 위치 가져오기
     Pose2d currentPose = m_odometry.getPoseMeters();
 
     // 🔹 SmartDashboard 업데이트
     SmartDashboard.putNumber("Pose X", currentPose.getX());
     SmartDashboard.putNumber("Pose Y", currentPose.getY());
     SmartDashboard.putNumber("Pose Rotation (deg)", currentPose.getRotation().getDegrees());
 
     // 🔹 Shuffleboard에서 기존 Entry 값을 업데이트 (반복 등록 X)
     poseXEntry.setDouble(currentPose.getX());
     poseYEntry.setDouble(currentPose.getY());
     poseRotationEntry.setDouble(currentPose.getRotation().getDegrees());
         
     fieldSim.setRobotPose(m_odometry.getPoseMeters());
   
    }// 주행 중 일때, Gps가 계속해서 업데이트 되어 "현재 위치를 실시간으로 갱신하는것 과 같습니다 "

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();//현재 로봇의 위치(좌표와 방향)를 반환합니다 
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  //로봇의 오도메트리를 특정 위치(pose)로 재설정합니다.
  //자율 주행 모드 시작 시 초기 위치를 설정할 때 사용됩니다.
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  //실제 작동변화코드 
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // 입력된 속도를 실제 최대 속도에 맞게 변환
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    // 필드 기준 주행일 경우, 자이로 센서를 이용하여 실제 주행 방향 계산
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
         // 각 모듈에 원하는 상태를 설정
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * 멈춘다 정지 
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);   //====> 이거 수정 필요
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  //자이로 센서를 재설정하여 로봇의 현재 방향을 0으로 만듭니다.
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  //자이로 센서의 데이터를 사용하여 로봇의 현재 방향(각도)을 반환합니다.
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  //로봇이 회전하는 속도(각속도)를 반환합니다.
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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

private PIDController pidControllerX = new PIDController(1.0, 0.0, 0.0); // 예시 값
private PIDController pidControllerY = new PIDController(1.0, 0.0, 0.0); // 예시 값

    // ... (다른 메서드)

    public void driveWithPID(double targetX, double targetY, double currentXSpeed, double currentYSpeed) {
        double xOutput = pidControllerX.calculate(currentXSpeed, targetX);
        double yOutput = pidControllerY.calculate(currentYSpeed, targetY);
        drive(xOutput, yOutput, 0.0, true); // 기존 drive 메서드를 호출
    }
}