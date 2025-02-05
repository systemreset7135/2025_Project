// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;//구동 
  private final SparkMax m_turningSpark;//회전 

  private final RelativeEncoder m_drivingEncoder;//주행거리 측정
  private final AbsoluteEncoder m_turningEncoder;//바퀴 현재 각도 

  private final SparkClosedLoopController m_drivingClosedLoopController;//주행 pid 
  private final SparkClosedLoopController m_turningClosedLoopController;// 조향 모터 pid 

  private double m_chassisAngularOffset = 0;//로봇 몸통, 각 모듈 0도 
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d()); //현재 모듈이 목표로 하는 상태 

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

  // SPARKS를 초기 상태로 리셋하고, 미리 정해진 설정(Configs.MAXSwerveModule)을 적용하여
  // 안전한 파라미터 상태로 만들고, 이후 전원 사이클 후에도 설정이 유지되도록 합니다.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
    //완전 모터, 엔코더와 절대 엔코더(조향각 센서) 공장 초기 설정 값과 같습니다 
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
   // 현재 주행 속도(velocity)와 조향 각도(엔코더 측정값에서 오프셋 보정)를 사용하여 모듈의 현재 상태(SwerveModuleState)를 반환합니다.
   //자동차의 속도계와 조향 센서를 통해 현재 자동차의 속도와 방향을 읽어오는 것과 같습니다.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // 자동차의 주행 거리를 기록하는 주행계와, 바퀴의 회전 각도를 읽어오는 센서를 통해 현재 차량이 얼마나 이동했는지를 확인하는 것과 비슷합니다.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the 원하는 위치 
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // 원하는 상태에 체시 오프셋을 적용하여 보정
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // 현재 조향 상태와 비교해 불필요하게 90도 이상 회전하는 것을 최적화(최소 회전각으로 목표를 달성)
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

     // 계산된 목표 속도와 각도를 폐쇄 루프 제어기를 통해 모터에 명령 전달
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
    //자동차의 조향 시스템에서 운전자가 핸들을 돌려 원하는 방향으로 조향하는 것과 같이, 목표 방향과 속도를 미리 계산 한후, 모터에 정확하게 명령을 전달 하는 과정입니다 
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
