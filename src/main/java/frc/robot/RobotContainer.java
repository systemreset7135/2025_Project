// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 여기는 로봇 두뇌 입니다 
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevatorSetpointCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.RRTPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.ElevatorSetpointCommand;
import frc.robot.utils.FieldMap;
import frc.robot.utils.RRTPlanner;
import java.util.Map;
import java.util.List;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
  private final Field2d field = new Field2d();
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();//바퀴 
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();//엘리베이터 
  private final ElevatorSetpointCommand elevatorCommand = new ElevatorSetpointCommand(m_elevator);
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);//조종기 
  private GenericEntry driveSpeedEntry;
  public RobotContainer() {
    
    configureButtonBindings();// 특정 버튼이 눌렸을 때 어떤 동작을 할지 미리 설정합니다.
    
    configureShuffleboardWidgets();//test

    SmartDashboard.putData("Field", field);

    m_elevator.setDefaultCommand(elevatorCommand);
    // 항상 실행되는 기본 주행 명령 설정 시동 일반모드 
    m_robotDrive.setDefaultCommand(
        
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),//Y 축 조이스틱
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),//X 축 조이스틱
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),// 회전축 조이스틱
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kDriverSetXIndex)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),//주차시 바퀴 방향 고정
            m_robotDrive));
    
    // RobotContainer.java (혹은 적절한 위치)
    JoystickButton elevatorButton = 
    new JoystickButton(m_driverController, OIConstants.kDriverElevatorL1Index);
// 'm_elevator' 서브시스템 REQUIRE 하지 않게!
elevatorButton.onTrue(new InstantCommand(() -> elevatorCommand.buttonPressed()));

    
  }
  public double getDriveSpeed() {
    return driveSpeedEntry.getDouble(0.0);
}
private void configureShuffleboardWidgets() {
  // "Test" 탭을 생성합니다.
  ShuffleboardTab testTab = Shuffleboard.getTab("Test");

  // "Drive Speed" 슬라이더 추가: 기본값 0.0, 범위 0 ~ 1
  driveSpeedEntry = testTab.add("Drive Speed", 0.0)
                            .withWidget(BuiltInWidgets.kNumberSlider)
                            .withProperties(Map.of("min", 0, "max", 1))
                            .getEntry();
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

//마치 자율주행 자동차가 출발하기 전에 내비게이션에 경로를 입력하고, 그 경로를 따라 주행하며, 도착하면 스스로 멈추는 것과 같습니다.
/** 📌 자동 주행 (Autonomous) 실행 */
public Command getAutonomousCommand() {
  Pose2d goalPose = new Pose2d(3.620, 5.637, Rotation2d.fromDegrees(-142.582));

    // ❌ 모든 탐색된 노드가 아니라 ✔ 최적 경로만 표시
    List<Pose2d> rrtPath = RRTPlanner.generateRRTPath(m_robotDrive.getPose(), goalPose, FieldMap.getObstacles());
    
    // 🔥 필드에서 실제 이동할 경로만 표시!
    List<Pose2d> optimizedPath = RRTPlanner.getOptimizedPath(rrtPath);
    field.getObject("RRT Path").setPoses(optimizedPath);

    return m_robotDrive.followRRTPath(goalPose);
}
}