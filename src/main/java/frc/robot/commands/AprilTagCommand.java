package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystemPigeon2;
import frc.robot.subsystems.AprilTagSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AprilTagCommand extends Command {
    private final DriveSubsystemPigeon2 driveSubsystem;
    private final AprilTagSubsystem limelightSubsystem;
    private boolean obstacleDetected = false;

    public AprilTagCommand(DriveSubsystemPigeon2 driveSubsystem, AprilTagSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        obstacleDetected = false;
    }

    @Override
    public void execute() {
        if (limelightSubsystem.hasValidTarget()) {
            if (!obstacleDetected) {
                System.out.println("Obstacle detected!");
                obstacleDetected = true;
            }
            double targetX = limelightSubsystem.getTargetX();
            double kP = 0.05; // 비례 제어 상수
            double speed = 0.2; // 천천히 따라가도록 설정

            // 물체의 x 좌표를 기반으로 회전 속도 계산
            double rotationSpeed = kP * targetX;

            // 물체를 피하기 위한 회피 동작
            driveSubsystem.drive(0, speed, rotationSpeed, true, true); // 오른쪽으로 회피
        } else {
            if (obstacleDetected) {
                System.out.println("Obstacle cleared.");
                obstacleDetected = false;
            }
            // 물체가 없으면 앞으로 천천히 이동
            driveSubsystem.drive(-0.2, 0, 0, true, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return false; // 명령이 계속 실행되도록 함
    }
}
