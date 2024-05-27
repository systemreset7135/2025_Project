
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
//inshallah eveything works
 
package frc.robot;
 
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
 
  private RobotContainer m_robotContainer;
  NetworkTable limelightTable;
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Shuffleboard.getTab("Example tab").add(m_gyro);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
  }
 
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("time matchtime", Timer.getMatchTime());
    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);
    SmartDashboard.putNumber("pidgeon 2.0 test", pigeon.getAngle());
  }
 
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
 
  @Override
  public void disabledPeriodic() {}
 
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
 
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
 
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
 
   
   
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
 
   
  }
 
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
 
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleopPeriodic();
    SmartDashboard.putString("tid", limelightTable.getEntry("tid").getDoubleArray(new double[6]).toString());
    SmartDashboard.putNumber("tx", limelightTable.getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("ty", limelightTable.getEntry("ty").getDouble(0));
    SmartDashboard.putNumber("tv", limelightTable.getEntry("tv").getDouble(0));
    SmartDashboard.putNumber("tlong", limelightTable.getEntry("tlong").getDouble(0));
    if (limelightTable.getEntry("tv").getDouble(0) == 1 && limelightTable.getEntry("tlong").getDouble(0) < 500) {
      // 회전을 멈춥니다
      System.out.println("Spots Target!");
     
      // AprilTag와의 거리를 가져옵니다
      double distance = limelightTable.getEntry("tlong").getDouble(0);
 
      // 만약 AprilTag와의 거리가 2m보다 작으면 로봇을 멈춥니다
      // 이후에 필요한 경우 움직임을 추가하실 수 있습니다
      if (distance > LimelightConstants.kMaximumDistance && distance < LimelightConstants.kMinimumDistance) {
        System.out.println("Reached Destinated Area");
          // 여기에 추가 동작을 입력하세요
      }
      else if(distance < LimelightConstants.kMaximumDistance){
         
        System.out.println("Too far"); // 전진 속도를 필요에 따라 조절하세요
      }
      else if(distance > LimelightConstants.kMinimumDistance){
        System.out.println("Too close");
      }
    }
    else {
        System.out.println("No Target");
    }
  }
 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
 
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
 