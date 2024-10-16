package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystemPigeon2;

public class AprilTagCommand extends Command {
    private final AprilTagSubsystem aprilTagSubsystem;
    private final DriveSubsystemPigeon2 m_robotDrive;
    private static final double TARGET_DISTANCE = 1.0;

    public AprilTagCommand(AprilTagSubsystem aprilTagSubsystem, DriveSubsystemPigeon2 m_robotDrive) {
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.m_robotDrive = m_robotDrive;
        addRequirements(aprilTagSubsystem);
    }

    @Override
    public void initialize() {
        // 초기화 코드
    }

    @Override
    public void execute() {
        // AprilTagSubsystem에서 데이터를 가져와서 필요한 작업 수행
        // 예: 인식 여부에 따라 다른 행동 수행
        if (aprilTagSubsystem.isTagDetected()) {
           double ApriltagDistance = aprilTagSubsystem.getTagDistance();
           double distanceError = ApriltagDistance - TARGET_DISTANCE;
           
           double speed = calculateSpeed(distanceError);
           
           m_robotDrive.drive(-0.08, 0, 0, true, true);
            // 태그가 인식되면 특정 작업 수행
        } else {
            m_robotDrive.drive(0, 0, 0, true, true);
            // 태그가 인식되지 않으면 다른 작업 수행
        }
    }
        private double calculateSpeed(double distanceError) {
            // 간단한 P 제어기 사용 (비례 제어기)
            double kP = 0.05; // 비례 상수 (조정 필요)
            return kP * distanceError;
        }
        
    
    

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false; // 명령이 언제 종료될지 정의
    }
}
