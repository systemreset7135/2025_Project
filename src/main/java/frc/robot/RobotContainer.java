// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
 
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AprilTagSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbingCmd;
// import frc.robot.commands.AprilTagCommand;
import frc.robot.commands.TeleopShootCmd;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.AprilTagCommand;
import frc.robot.commands.AutoShootingCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystemPigeon2;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
 
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystemPigeon2 m_robotDrive = new DriveSubsystemPigeon2();
  private final ShooterSubsystem m_ShootSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  PS4Controller n_driverController = new PS4Controller(OIConstants.kDriverControllerPortSec);
 
  // 발사 메커니즘 모터
 
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
 
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        // 설정 추가
    m_ShootSubsystem.setDefaultCommand(new TeleopShootCmd(m_ShootSubsystem, "stopped"));
    m_ClimberSubsystem.setDefaultCommand(new ClimbingCmd(m_ClimberSubsystem, "stopped"));
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
    PS4Controller functionController = m_driverController;
    if(OIConstants.kDoublePlayer){
      functionController = n_driverController;
    }
    new JoystickButton(m_driverController, OIConstants.kDriverSetXButtonIndex)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // new JoystickButton(functionController, OIConstants.kDriverShootButtonIndex).toggleOnTrue(new TeleopShootCmd(m_ShootSubsystem, "out"));
    Command teleopShootCommand = new TeleopShootCmd(m_ShootSubsystem, "out");
    if(teleopShootCommand.isFinished() == true){
      new JoystickButton(functionController, OIConstants.kDriverShootButtonIndex).toggleOnTrue(teleopShootCommand);
    }
    new JoystickButton(functionController, OIConstants.kDriverIntakeButtonIndex).whileTrue(new TeleopIntake(m_ShootSubsystem, "in"));
    new JoystickButton(functionController, OIConstants.kDriverClimbUpButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "up"));
    new JoystickButton(functionController, OIConstants.kDriverClimbDownButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "down"));
    new JoystickButton(functionController, OIConstants.kDriverAutoClimbButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "auto"));
   
   
    new JoystickButton(m_driverController, OIConstants.kDriverSetXButtonIndex)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
 
   
    // new JoystickButton(functionController, OIConstants.kDriverShootButtonIndex).toggleOnTrue(new TeleopShootCmd(m_ShootSubsystem, "out"));
    Command teleop1ShootCommand = new TeleopShootCmd(m_ShootSubsystem, "out");
    if(teleop1ShootCommand.isFinished() == true){
      new JoystickButton(m_driverController, OIConstants.kDriverShootButtonIndex).toggleOnTrue(teleopShootCommand);
    }
    new JoystickButton(m_driverController, OIConstants.kDriverIntakeButtonIndex).whileTrue(new TeleopIntake(m_ShootSubsystem, "in"));
    new JoystickButton(m_driverController, OIConstants.kDriverClimbUpButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "up"));
    new JoystickButton(m_driverController, OIConstants.kDriverClimbDownButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "down"));
    new JoystickButton(m_driverController, OIConstants.kDriverAutoClimbButtonIndex).whileTrue(new ClimbingCmd(m_ClimberSubsystem, "auto"));
    new JoystickButton(m_driverController, OIConstants.kDriverResetGyroButtonIndex).whileTrue(new RunCommand(() -> m_robotDrive.resetGyro(), m_robotDrive));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command gyroreset = new RunCommand(
    //   () -> {
    //     m_robotDrive.resetGyro();
    //       });
    
      
        return new SequentialCommandGroup(
          // gyroreset,
          new AprilTagCommand(aprilTagSubsystem, m_robotDrive)
            
        );
   
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecond,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);
 
  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       config);
 
  //   var thetaController = new ProfiledPIDController(
  //       AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
 
  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       m_robotDrive::getPose, // Functional interface to feed supplier
  //       DriveConstants.kDriveKinematics,
 
  //       // Position controllers
  //       new PIDController(AutoConstants.kPXController, 0, 0),
  //       new PIDController(AutoConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       m_robotDrive);
 
  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
 
  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  // }
 
 
 
 
 
 
// ------------------------------------------------------------------------------------
 
 
// middle Command 中间
  // public Command getAutonomousCommand() {
  //   Wheelinitial();
  //   Command driveForwardCommand = new RunCommand(
  //     () -> {
  //       System.out.println("Forward start");
  //       m_robotDrive.drive(0.085, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(2);
  //   Command stopCommand = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(0.25);
  //   Command autoShootCommand = new InstantCommand(() -> m_ShootSubsystem.shoot("out")).withTimeout(0);
 
  //   Command driveForwardCommandd = new RunCommand(
  //     () -> m_robotDrive.drive(0.08, 0, 0 , true, true),
  //     m_robotDrive
 
  //     )
  //     .withTimeout(2);
  //   Command stoppCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, 0 , true, true),
  //     m_robotDrive
 
  //     )
  //     .withTimeout(0.25);
  //   Command rotationCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, 0.55 , true, true),
  //     m_robotDrive
 
  //     )
  //     .withTimeout(2);
  //   // Command shootingCommand = new ShootingCmd(m_ShootSubsystem, "auto");
 
  //   // MIDDLE AUTO
  //   return new SequentialCommandGroup(driveForwardCommand, new WaitCommand(2), stopCommand, autoShootCommand, new WaitCommand(2), driveForwardCommandd, new WaitCommand(2), stoppCommand, rotationCommand);
   
   
  // };
  // --------------------------------------------------
 
 
  // // // side Comand 左大（别动）
  // public Command getAutonomousCommand() {
  //   Wheelinitial();
  //   Command drivesideCommand = new RunCommand(
  //   () -> {
     
  //     m_robotDrive.drive(0.085, 0.085, 0 , true, true);
  //   },
  //   m_robotDrive
 
  //   )
  //   .withTimeout(1);
   
  //   Command rotationCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, 0.22 , true, true),
  //     m_robotDrive
 
  //     )
  //     .withTimeout(2);
  //   Command stopCommand1 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
 
  //     .withTimeout(1);
  //      Command stopCommand2 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(0.25);
 
  //   Command autoShootCommand = new InstantCommand(() -> m_ShootSubsystem.shoot("out")).withTimeout(1);
 
  //   Command rotateCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, -1.08 , true, true),
  //     m_robotDrive
     
  //     )
  //     .withTimeout(1);
     
  //     Command backoutCommand = new RunCommand(
  //       () -> m_robotDrive.drive(-0.25, 0, 0 , true, true),
  //       m_robotDrive
       
  //       )
  //       .withTimeout(8);
       
       
       
       
       
  // //       // return new SequentialCommandGroup(drivesideCommand, stopCommand2,new WaitCommand(2), autoShootCommand, new WaitCommand(2), rotationCommand, stopCommand1, new WaitCommand(1.5), backoutCommand);
  //      return new SequentialCommandGroup(autoShootCommand, new WaitCommand(2),  rotateCommand, stopCommand2, backoutCommand);
  //   };
 
   //side Comand 左大修改
  // public Command getAutonomousCommand() {
  //   Wheelinitial();
  //   Command stopCommand1 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
 
  //     .withTimeout(1);
  //      Command stopCommand2 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(0.25);
 
 
  //   Command drivewardXfirst = new RunCommand(
  //   () -> {
     
  //     m_robotDrive.drive(0.05, 0.085, 0 , true, true);
  //   },
  //   m_robotDrive
 
  //   )
  //   .withTimeout(0.7);
  //   Command drivewardYfirst = new RunCommand(
  //   () -> {
     
  //     m_robotDrive.drive(0, 0.05, 0 , true, true);
  //   },
  //   m_robotDrive
 
  //   )
  //   .withTimeout(0.1);
   
   
 
  //   Command autoShootCommand = new InstantCommand(() -> m_ShootSubsystem.shoot("out")).withTimeout(1);
 
  //   Command rotateCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, -1.08 , true, true),
  //     m_robotDrive
     
  //     )
  //     .withTimeout(1);
     
  //     Command backoutCommand = new RunCommand(
  //       () -> m_robotDrive.drive(-0.14, 0, 0 , true, true),
  //       m_robotDrive
       
  //       )
  //       .withTimeout(8);
       
       
       
       
       
  //       // return new SequentialCommandGroup(drivesideCommand, stopCommand2,new WaitCommand(2), autoShootCommand, new WaitCommand(2), rotationCommand, stopCommand1, new WaitCommand(1.5), backoutCommand);
  //    return new SequentialCommandGroup(drivewardXfirst, new WaitCommand(0.25), drivewardYfirst, new WaitCommand(1.5), stopCommand1, new WaitCommand(1.5),autoShootCommand, new WaitCommand(1.5),  rotateCommand, stopCommand2, backoutCommand);
  // };
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  // ---------------------------------------------------
 
  // //side Comand 右小
  // public Command getAutonomousCommand() {
  //   Wheelinitial();
  //   Command stopCommand1 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(1);
 
  //   Command stopCommand2 = new RunCommand(
  //     () -> {
  //       System.out.println("STOP");
  //       m_robotDrive.drive(0, 0, 0 , true, true);
  //     },
  //     m_robotDrive
 
  //     )
  //     .withTimeout(1);
   
  //   Command drivesideCommand = new RunCommand(
  //   () -> {
     
  //     m_robotDrive.drive(0.085, 0.085, 0 , true, true);
  //   },
  //   m_robotDrive
 
  //   )
  //   .withTimeout(1);
  //   Command rotationCommand = new RunCommand(
  //     () -> m_robotDrive.drive(0, 0, 0.22 , true, true),
  //     m_robotDrive
 
  //     )
  //     .withTimeout(2);
 
 
  //   Command autoShootCommand = new InstantCommand(() -> m_ShootSubsystem.shoot("out")).withTimeout(0);
 
  //   Command backoutCommand = new RunCommand(
  //       () -> m_robotDrive.drive(0.1, 0, 0 , true, true),
  //       m_robotDrive
 
  //       )
  //       .withTimeout(3);
  //   return new SequentialCommandGroup(drivesideCommand, stopCommand1, autoShootCommand, new WaitCommand(2), rotationCommand, stopCommand2, backoutCommand);
  //    移动到设定位置， 射，转，往前
  // };
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
  public void teleopPeriodic() {
   
  // double maxForwardSpeed = 0.1; // 최대 전진 속도를 50%로 설정
  // double maxRotationSpeed = 0.1;
 
 
 
  // // 조이스틱 입력을 가져옴
  // double forward = m_driverController.getRawAxis(1); // Forward는 조이스틱을 아래로 향할수록 양수
  // double strafe = m_driverController.getRawAxis(0);   // 오른쪽으로 향할수록 양수
 
  // double rotation = m_driverController.getRawAxis(2); // 시계 방향으로 회전할수록 양수
 
  // // 최대 전진 속도 적용
  // forward = forward * maxForwardSpeed;
  // strafe = strafe * maxForwardSpeed;
  // rotation = rotation * maxRotationSpeed;
// 조이스틱 입력에 따라 바퀴를 회전시키거나 직진/후진하도록 모터를 제어(2번째)
// if (Math.abs(strafe) > 0.1) {
//   // 좌우 이동 값이 일정 값 이상일 때는 바퀴를 회전시킴
//   // 예를 들어, 오른쪽으로 이동하는 경우
//   m_robotDrive.drive(0, 0, strafe, true, true);
// } else {
//   // 좌우 이동 값이 일정 값 미만일 때는 바로 해당 방향으로 이동
//   // 전진
//   if (forward > 0.1) {
//       m_robotDrive.drive(forward, 0, 0, true, true);
//   }
//   // 후진
//   else if (forward < -0.1) {
//       m_robotDrive.drive(forward, 0, 0, true, true);
//   }
//   // 정지
//   else {
//       m_robotDrive.drive(0, 0, 0, true, true);
//   }
// }
 
  // 조이스틱 입력에 deadbend apply
  // forward = applyDeadband(forward, OIConstants.kDriveDeadband);
  // strafe = applyDeadband(strafe, OIConstants.kDriveDeadband);
  // rotation = applyDeadband(rotation, OIConstants.kDriveDeadband);
 
 
    //  조이스틱 입력의 크기가 0 또는 매우 작을 때 바퀴를 정면으로 설정
// if (Math.abs(forward) < 0.1 && Math.abs(strafe) < 0.1 && Math.abs(rotation) < 0.1) {
//     // 정면으로 설정하는 코드 추가
//     // m_robotDrive.drive(0, 0, 0, true, true); // 모든 바퀴를 멈추도록 설정
//     Wheelinitial();
// } else {
//     // 조이스틱 입력을 사용하여 로봇 구동
//     m_robotDrive.drive(forward, strafe, rotation, true, true);
//     m_robotDrive.drive(-forward, strafe, rotation, true, true);
// }
  }
 
 
 
 
  private void Wheelinitial() {
    MAXSwerveModule frontRightModule = m_robotDrive.getFrontRightModule();
    MAXSwerveModule frontLeftModule = m_robotDrive.getFrontLeftModule();
    MAXSwerveModule rearLeftModule = m_robotDrive.getRearLeftModule();
    MAXSwerveModule rearRighttModule = m_robotDrive.getRearRightModule();
 
    SwerveModuleState zeroState = new SwerveModuleState(0, frontRightModule.getState().angle);
 
    frontRightModule.setDesiredState(zeroState);
    frontLeftModule.setDesiredState(zeroState);
    rearLeftModule.setDesiredState(zeroState);
    rearRighttModule.setDesiredState(zeroState);
  }
 
 
 
}
 