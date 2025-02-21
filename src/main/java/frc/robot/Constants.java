// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {//로봇 주행관련 상수 
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 0.8; //3.0도 나쁘지 않아  //로봇 최대 속도 
    public static final double kMaxAngularSpeed = 0.2 * Math.PI; // radians per second 로봇 회전 속도 

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26); // 좌우 바퀴 사이 값 
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26); //앞 뒤 바퀴 사이 값 
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(//네 바퀴에 상대적인 위히를 지정 
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians 로봇 섀시 각도오차 보정 
    public static final double kFrontLeftChassisAngularOffset = Units.degreesToRadians(180);  
    public static final double kBackRightChassisAngularOffset = Units.degreesToRadians(0);//backright    
    public static final double kFrontRightChassisAngularOffset = Units.degreesToRadians(0); //front right     
    public static final double kBackLeftChassisAngularOffset = Units.degreesToRadians(180); //backleft  

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 17;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 15;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 18;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants { //드라이브 모터와 관련되어 있는것 
    
    public static final int kDrivingMotorPinionTeeth = 13; //작은 모터에 치아수, 값에 따라 로봇에 속도가 달라집니다 

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60; // 모터의 자유 속도 60으로 나누어 초당 회전수, 모터 최대 출력 
    public static final double kWheelDiameterMeters = 0.0762; //로봇 바퀴 지름 
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; //바퀴 둘래 
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinio0
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); //모터와 바퀴사의 감속비, 바퀴에 전달되는 회전수가 결정됨 
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) //모터의 자유 속도(RPS)를 바퀴의 둘레와 감속비를 고려하여 바퀴가 실제 구동할 수 있는 최대 속도를 계산합니다.
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {//운전자 인터 페이스 
    public static final int kDriverControllerPort = 0; //usb 조종기 포트번호 
    public static final double kDriveDeadband = 0.1; // 조종기 미세한 움지김 무시 
    public static final int kDriverElevatorL1Index = 1; //엘리베이터 
    // public static final int kDriverElevatorL2Index = 2; //엘리베이터 
    // public static final int kDriverElevatorL3Index = 4; //엘리베이터 
    public static final int kDriverResetGyroButtonIndex = 3; //자이로 리셋 
    public static final int kDriverSetXIndex = 2; //모양 주차 
  }

  public static final class PathConstants{
    public static final long KAdStarTimeout = 20000;
  }

  public static final class AutoConstants { //자율주행모드 
    public static final double kMaxSpeedMetersPerSecond = 3; //자율주행모드 최대 속도 
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;//최대 가속도 
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;//최대 각속도 
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //최대 각가속도 
    
    //자율 주행 경로 추종(trajectory following) 시 위치 및 각도 제어에 사용되는 PID 상수입니다.
    //각각 X, Y 방향과 로봇의 회전(Theta)에 대한 제어에 사용됩니다.
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    //
    // TrapezoidProfile.Constraints를 사용하여, 로봇의 회전(각도) 제어에 대한 최대 속도와 가속도 제약 조건을 정의합니다
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {//Neo모터 
    public static final double kFreeSpeedRpm = 5676;//모터의 자유 속도 
  }
  public class ElevatorConstants {
    public static final double MIN_HEIGHT = 0.0; // 엘리베이터가 이동할 수 있는 최소 높이
    public static final double MAX_HEIGHT = 48.0; // 엘리베이터가 이동할 수 있는 최대 높이
    public static final int Elevator1CanId = 23;//엘리베이터 모터 
    public static final int Elevator2CanId = 24;//shibal
    public static final double kElevatorHighSetpoint = 48.0; //엘리베이터 최대 높이
    public static final double kP = 0.1; // Proportional gain
    public static final double kI = 0; // Integral gain
    public static final double kD = 0; // Derivative gain
    public static final double kTolerance = 0.5; // Acceptable position error in inches
    public static final double[] kSetpoints = {0.0, 20.0, 30.0, 40.0}; // Target heights in inches
  }
  public static class ShooterConstants {
    public static final int kFrontRightShootPort = 5;
    public static final int kFrontLeftShootPort = 4;
    public static final int kRearRightShootPort = 2;
    public static final int kRearLeftShootPort = 1;
    public static final double kShootChargeTime = 1.5;
    public static final double kShootSpeed = 1;
    public static final double kIntakeSpeed = -0.5;
  }
}