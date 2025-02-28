package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.DriveFeedforwards;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

public class SAutocmd extends Command {
    private final DriveSubsystem drive;
    private Command followPathCommand;
    private static final double MAX_VELOCITY = 1.0; // 지금 설정된 최대속도 (m/s)
    private static final double MAX_ACCELERATION = 1.0; // 지금 설정된 최대가속도 (m/s^2)
    private final Field2d fieldSim; 

    private Pose2d startPose;
    private Pose2d endPose; 
    private final String pathName;

    public SAutocmd(DriveSubsystem drive, String pathName) {
        this.drive = drive;
        this.fieldSim = drive.getFieldSim();
        this.pathName = pathName;
        addRequirements(drive); // 드라이브 서브시스템을 이 명령에 필요로 함
    }

    @Override
    public void initialize() {
        startPose = drive.getPose();
        SmartDashboard.putNumber("Robot Start X", startPose.getX());
        SmartDashboard.putNumber("Robot Start Y", startPose.getY());
        SmartDashboard.putNumber("Robot Start Rot (deg)", startPose.getRotation().getDegrees());

        try {
            // 1) PathPlanner 경로 로드 ("path 1" 파일 사용)
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName); 
            if (path == null) {
                System.out.println("[SAutocmd] 경로를 찾을 수 없음!");
                return;
            }

            drive.drawPathOnField("point", path);


            // 2) 경로로부터 궤적 생성
            RobotConfig robotConfig = RobotConfig.fromGUISettings(); 
            if (robotConfig == null) {
                System.out.println("[SAutocmd] RobotConfig을 로드할 수 없음!");
                return;
            }

            PathPlannerTrajectory trajectory = path.generateTrajectory(
                new ChassisSpeeds(0.0, 0.0, 0.0), // 초기 속도 0 안정적으로 시작
                path.getInitialHeading(), 
                robotConfig     
            );

            

            // 3) 오도메트리를 경로 시작 포즈로 리셋
            Pose2d startPose = trajectory.getInitialPose();
            drive.resetOdometry(startPose);

            // 4) 경로 추종 명령 생성 (PPHolonomicDriveController 사용)
            followPathCommand = new FollowPathCommand(
                path,
                drive::getPose, // 현재 로봇 위치 제공
                drive::getChassisSpeeds, // 현재 섀시 속도 제공
                (speeds, feedforwards) -> {
                    // 섀시 속도를 스웨브 모듈 상태로 변환하고 적용
                    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
                    drive.setModuleStates(moduleStates);
                },
                new PPHolonomicDriveController(
                    new com.pathplanner.lib.config.PIDConstants(
                        Constants.AutoConstants.kPXController, // X 방향 PID (기본값 1.0)
                        Constants.AutoConstants.kIXController, // I 값 (기본값 0.0)
                        Constants.AutoConstants.kDXController  // D 값 (기본값 0.0)
                    ),
                    new com.pathplanner.lib.config.PIDConstants(
                        Constants.AutoConstants.kPYController, // Y 방향 PID (기본값 1.0)
                        Constants.AutoConstants.kIYController, // I 값 (기본값 0.0)
                        Constants.AutoConstants.kDYController  // D 값 (기본값 0.0)
                    )
                ),
                robotConfig,
                () -> false, // 경로 뒤집기 방지 (PathPlanner의 회전 정보를 신뢰)
                drive
            );
            



            // 5) 경로 추종 명령 초기화
            followPathCommand.initialize();
            System.out.println("[SAutocmd] 경로 실행 시작: " + path.name);

        } catch (IOException | ParseException e) {
            System.err.println("[SAutocmd] 경로 파일 로드 실패: " + e.getMessage());
            return;
        }
    }

    @Override
    public void execute() {
        if (followPathCommand != null) {
            followPathCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (followPathCommand != null) {
            return followPathCommand.isFinished();
        }
        return true; // 경로가 없으면 즉시 종료
    }

    @Override
    public void end(boolean interrupted) {
        if (followPathCommand != null) {
            followPathCommand.end(interrupted);
        }
        
        endPose = drive.getPose();
        System.out.println("[SAutocmd] 마지막 위치 - X: " + endPose.getX() + 
                          ", Y: " + endPose.getY() + 
                          ", Rotation: " + endPose.getRotation().getDegrees() + "도");
        
        Pose2d desiredEndPose = new Pose2d(
        endPose.getX(), // X 좌표 유지
        endPose.getY(), // Y 좌표 유지
        Rotation2d.fromDegrees(180) // 원하는 회전 (예: 180도)
    );
    drive.resetOdometry(desiredEndPose); // 오도메트리 강제 업데이트
        // 로봇 정지
        drive.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
    }

  
    
}