// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//여기는 실행 관리자 입니다 
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;//자율주행 모드에서 실행할 명령 저장 변수 

  private RobotContainer m_robotContainer;// 로봇 컨테이너 와 연결 


  @Override
  public void robotInit() {
   //로봇 처음 켰을때 실행 function 
    m_robotContainer = new RobotContainer();//두뇌 초기화 합니다..그리고 시작 
    
  }

  
  @Override
  public void robotPeriodic() {
    //로봇의 모든 모드(자율, 텔레옵, 테스트, 비활성화)에서 20ms마다 호출됩니다.
    CommandScheduler.getInstance().run();
  }

  //안전에 관련된 코드 
  //로봇이 경기를 준비하는 동안, 센서를 리셋하거나 상태를 초기화하는 작업을 이곳에서 처리할 수 있습니다.
  @Override
  public void disabledInit() {
    //비활성 상태 코드 
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); // 자율 명령 스케줄
    }
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //자율 모드에서 20ms마다 호출되지만, 여기서는 별도의 로직이 없습니다. 
    //대부분의 작업은 CommandScheduler가 robotPeriodic()에서 처리하기 때문에 이 메소드에는 추가 코드를 넣지 않아도 됩니다.

  }

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();// 자율 명령어 취소, 이는 자율 동작이 계속 실행되지 않도록 보장합니다 
    }
  }


  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll(); //모든명령어를 취소 하여, clean 한 상태
  }

 
  @Override
  public void testPeriodic() {}
}