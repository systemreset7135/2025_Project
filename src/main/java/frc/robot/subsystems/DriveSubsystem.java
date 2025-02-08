package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.FieldMap;
import frc.robot.utils.RRTPlanner;
import java.util.List;

public class DriveSubsystem extends SubsystemBase {
    // 4개의 스웨브 모듈 설정
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset
    );

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset
    );

    // 자이로 센서 (Pigeon2)
    private final Pigeon2 m_gyro = new Pigeon2(0);

    // 오도메트리 (위치 추적)
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
    );

    public DriveSubsystem() {
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Rotation (deg)", getPose().getRotation().getDegrees());
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }
        );

        Pose2d currentPose = m_odometry.getPoseMeters();
        SmartDashboard.putNumber("Pose X", currentPose.getX());
        SmartDashboard.putNumber("Pose Y", currentPose.getY());
        SmartDashboard.putNumber("Pose Rotation (deg)", currentPose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds chassisSpeeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeedDelivered,
                      ySpeedDelivered,
                      rotDelivered,
                      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
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

    /** 📌 특정 위치로 이동하는 함수 (RRT 적용) */
    public void driveToPosition(Pose2d targetPose) {
        double xSpeed = targetPose.getX() - getPose().getX();
        double ySpeed = targetPose.getY() - getPose().getY();
        double rotation = targetPose.getRotation().getRadians() - getPose().getRotation().getRadians();

        drive(xSpeed, ySpeed, rotation, true);
    }
    public void logRRTPath(List<Pose2d> path) {
      if (path.size() < 2) return; // 경로가 너무 짧으면 출력하지 않음
  
      // ❌ 모든 노드가 아니라 ✔ "시작점과 도착점"만 SmartDashboard에 표시
      SmartDashboard.putNumber("RRT Path Start X", path.get(0).getX());
      SmartDashboard.putNumber("RRT Path Start Y", path.get(0).getY());
  
      SmartDashboard.putNumber("RRT Path Goal X", path.get(path.size() - 1).getX());
      SmartDashboard.putNumber("RRT Path Goal Y", path.get(path.size() - 1).getY());
  }
    /** 📌 RRT 기반 자율 주행 경로 실행 */
    public Command followRRTPath(Pose2d goalPose) {
        Pose2d startPose = getPose();
        List<Translation2d> obstacles = FieldMap.getObstacles();

        // RRT 경로 생성
        List<Pose2d> rrtPath = RRTPlanner.generateRRTPath(startPose, goalPose, obstacles);

        // RRT 경로를 따라 이동하는 명령 생성
        SequentialCommandGroup followPathCommand = new SequentialCommandGroup();
        for (Pose2d pose : rrtPath) {
            followPathCommand.addCommands(new RunCommand(
                () -> driveToPosition(pose),
                this
            ).withTimeout(1)); // 각 포인트마다 1초 제한
        }

        return followPathCommand.andThen(new InstantCommand(() -> drive(0, 0, 0, false)));
    }
}
