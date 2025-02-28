package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathGenerator;
import frc.robot.subsystems.Limelightsub;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * AdvancedAvoid (비동기):
 *  - 장애물 감지 시 FollowPathCommand를 interrupt하고,
 *    옆으로 회피 후, AD*로 새 경로를 비동기로 생성
 *  - Pathfinding.isNewPathAvailable() 확인 후 빌드
 */
public class AdvancedAvoid extends Command {
    public enum DriveMode {
        NORMAL,
        OBSTACLE_AVOIDANCE,
        PREPARE_PATH_GENERATION,
        WAITING_FOR_NEW_PATH,
        RETRY_PATH_GENERATION,
        FAILED
    }

    private final DriveSubsystem driveSubsystem;
    private final Limelightsub limelightSubsystem;
    private final DynamicPathGenerator pathGenerator;
    
    private final Command followPathCmdToCancel;
    private final Translation2d originalGoal;

    private DriveMode currentMode;
    private final Timer stateTimer = new Timer();

    private static final double AVOID_TIME = 1.0; // 회피 시간
    private static final double OBSTACLE_RECHECK_TIME = 2.0; // 장애물 재점검 시간
    private static final double NEAR_DISTANCE = 1.0; // 가까운 거리의 임계값 (미터)
    private static final double PATH_GENERATION_TIMEOUT = 10.0; // 경로 생성 타임아웃 시간 (초)
    private static final double EMERGENCY_DISTANCE = 0.5; // 긴급 정지 거리 (미터)
    private static final double SAFE_DISTANCE = 1.5; // 안전 거리 (미터)
    private static final double RETRY_DELAY = 1.0; // 재시도 간격 (초)

    private PathPlannerPath newPath = null;
    private int retryCount = 0;
    private int maxRetryAttempts = 2; // 기본 재시도 횟수

    private ExecutorService executor = Executors.newSingleThreadExecutor(); // 단일 스레드 풀

    private final Set<Subsystem> requirements = new HashSet<>();

    public AdvancedAvoid(
        DriveSubsystem driveSubsystem,
        Limelightsub limelightSubsystem,
        DynamicPathGenerator pathGenerator,
        Translation2d originalGoal,
        Command followPathCmdToCancel
    ) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.pathGenerator = pathGenerator;
        this.originalGoal = originalGoal;
        this.followPathCmdToCancel = followPathCmdToCancel;

        this.currentMode = DriveMode.NORMAL;
        requirements.add(driveSubsystem);
        requirements.add(limelightSubsystem);
    }

    @Override
    public void initialize() {
        stateTimer.reset();
        stateTimer.start();
        retryCount = 0;
        newPath = null;
        SmartDashboard.putString("Avoid/State", currentMode.toString());
        System.out.println("[AdvancedAvoid] 초기화. currentMode = " + currentMode);
    }

    @Override
    public void execute() {
        double elapsed = stateTimer.get();
        switch (currentMode) {
            case NORMAL:
                if (limelightSubsystem.isObstacleDetected()) {
                    if (followPathCmdToCancel != null && followPathCmdToCancel.isScheduled()) {
                        followPathCmdToCancel.cancel();
                        System.out.println(">>> FollowPathCommand CANCELED for obstacle");
                    }
                    transitionTo(DriveMode.OBSTACLE_AVOIDANCE, "장애물 감지");
                }
                break;

            case OBSTACLE_AVOIDANCE:
                double offsetX = limelightSubsystem.getOffsetX();
                double offsetY = limelightSubsystem.getOffsetY();
                double distance = limelightSubsystem.calculateDistance(limelightSubsystem.getTY());
                
                // 긴급 정지 로직
                if (distance < EMERGENCY_DISTANCE) {
                    driveSubsystem.drive(0.0, 0.0, 0.0, true); // 긴급 정지
                    transitionTo(DriveMode.FAILED, "긴급 정지 - 장애물 너무 가까움");
                    SmartDashboard.putNumber("Obstacle Distance", distance);
                    System.out.println("[AdvancedAvoid] Emergency Stop: Obstacle at " + distance + " meters");
                    return;
                }

                double ySpeed, xSpeed;
                
                // 안전 거리 유지 로직
                if (distance > SAFE_DISTANCE) {
                    // 안전 거리 이상이면 정상 회피 동작 수행
                    if (distance > NEAR_DISTANCE) {
                        ySpeed = offsetX > 0 ? -0.3 : 0.3;
                        xSpeed = Math.abs(offsetX) < 0.1 ? (offsetY < -0.1 ? -0.2 : (offsetY > 0.1 ? 0.2 : 0)) : 0;
                    } else {
                        ySpeed = offsetX > 0 ? -0.5 : 0.5;
                        xSpeed = Math.abs(offsetX) < 0.1 ? (offsetY < -0.1 ? -0.4 : (offsetY > 0.1 ? 0.4 : 0)) : 0;
                    }
                } else {
                    // 안전 거리 이하이면 속도를 줄여서 회피
                    ySpeed = offsetX > 0 ? -0.2 : 0.2;
                    xSpeed = Math.abs(offsetX) < 0.1 ? (offsetY < -0.1 ? -0.1 : (offsetY > 0.1 ? 0.1 : 0)) : 0;
                }

                if (elapsed < AVOID_TIME) {
                    // PID 제어 적용
                    double currentSpeed = driveSubsystem.getCurrentSpeed();
                    driveSubsystem.driveWithPID(xSpeed, ySpeed, currentSpeed * Math.signum(xSpeed), currentSpeed * Math.signum(ySpeed));
                } else {
                    driveSubsystem.drive(0.0, 0.0, 0.0, true);
                    if (elapsed > AVOID_TIME + OBSTACLE_RECHECK_TIME) {
                        if (limelightSubsystem.isObstacleDetected()) {
                            stateTimer.reset();
                        } else {
                            transitionTo(DriveMode.PREPARE_PATH_GENERATION, "장애물 해제");
                        }
                    }
                }
                
                SmartDashboard.putNumber("Obstacle Distance", distance);
                SmartDashboard.putNumber("Avoidance Speed", Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed));
                SmartDashboard.putString("Avoidance Direction", offsetX > 0 ? "Left" : "Right");
                System.out.printf("[AdvancedAvoid] Avoiding obstacle at distance %.2f, speed %.2f, direction %s\n", 
                                 distance, Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed), offsetX > 0 ? "Left" : "Right");
                break;

            case PREPARE_PATH_GENERATION:
                startPathGenerationAsync();
                transitionTo(DriveMode.WAITING_FOR_NEW_PATH, "AD* 초기화 완료");
                System.out.println("[AdvancedAvoid] Preparing new path from " + driveSubsystem.getPose().getTranslation());
                break;

            case WAITING_FOR_NEW_PATH:
                if (checkPathGenerationCompleted()) {
                    if (newPath != null) {
                        transitionTo(DriveMode.NORMAL, "경로 생성 성공");
                        System.out.println("[AdvancedAvoid] Path generation succeeded");
                    } else {
                        transitionTo(DriveMode.RETRY_PATH_GENERATION, "경로 빌드 실패");
                        System.out.println("[AdvancedAvoid] Path generation failed");
                    }
                } else if (elapsed > PATH_GENERATION_TIMEOUT) {
                    // 타임아웃 발생 시 임시 경로 생성
                    newPath = generateTemporaryPath(driveSubsystem.getPose(), originalGoal);
                    if (newPath != null) {
                        transitionTo(DriveMode.NORMAL, "임시 경로 생성");
                        System.out.println("[AdvancedAvoid] Path generation timed out. Using temporary path.");
                    } else {
                        // 임시 경로 생성도 실패하면 기본 회피 동작 수행
                        performDefaultAvoidance();
                        transitionTo(DriveMode.RETRY_PATH_GENERATION, "임시 경로 생성 실패, 기본 회피 후 재시도");
                        System.out.println("[AdvancedAvoid] Temporary path generation failed, default avoidance performed.");
                    }
                }
                break;

            case RETRY_PATH_GENERATION:
                if (elapsed > 2.0) {
                    handleRetry();
                }
                break;

            case FAILED:
                driveSubsystem.drive(0, 0, 0, true);
                break;
        }

        SmartDashboard.putString("Avoid/State", currentMode.toString());
    }

    @Override
    public boolean isFinished() {
        return false; // 이 명령은 상시 동작
    }

    @Override
    public void end(boolean interrupted) {
        executor.shutdownNow(); // 명령이 끝나면 스레드 풀 종료
        driveSubsystem.drive(0, 0, 0, true);
        System.out.println("[AdvancedAvoid] end() interrupted=" + interrupted);
    }

    private void transitionTo(DriveMode newMode, String reason) {
        System.out.println("[AdvancedAvoid] " + currentMode + " -> " + newMode + " : " + reason);
        currentMode = newMode;
        stateTimer.reset();
    }

    private void startPathGenerationAsync() {
        Pose2d pose = driveSubsystem.getPose();
        Translation2d startPos = pose.getTranslation();
        CompletableFuture.runAsync(() -> {
            pathGenerator.startPathGeneration(startPos, originalGoal, pose.getRotation());
            newPath = pathGenerator.buildPathWithConstraints();
        }, executor);
    }

    private boolean checkPathGenerationCompleted() {
        // newPath가 설정되었는지 확인
        return newPath != null;
    }

    // 임시 경로 생성 메서드
    private PathPlannerPath generateTemporaryPath(Pose2d currentPose, Translation2d goalPos) {
        PathConstraints constraints = new PathConstraints(2.0, 2.0, 2.0, 2.0, 12.0, false);
        GoalEndState goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(0.0));
        
        // PathPoint 생성 시 필요한 정보를 사용
        List<PathPoint> pathPoints = new ArrayList<>();
        
        // 시작 포인트
        RotationTarget startRotationTarget = new RotationTarget(0.0, currentPose.getRotation());
        pathPoints.add(new PathPoint(currentPose.getTranslation(), startRotationTarget));

        // 종료 포인트
        RotationTarget endRotationTarget = new RotationTarget(1.0, Rotation2d.fromDegrees(0.0));
        pathPoints.add(new PathPoint(goalPos, endRotationTarget));

        return PathPlannerPath.fromPathPoints(pathPoints, constraints, goalEndState);
    }

    // 기본 회피 동작
    private void performDefaultAvoidance() {
        // 예시: 직진 후 회전
        driveSubsystem.drive(0.5, 0.0, 0.0, true); // 앞으로 직진
        Timer.delay(1.0); // 1초 직진
        driveSubsystem.drive(0.0, 0.0, 0.5, true); // 오른쪽으로 회전
        Timer.delay(0.5); // 0.5초 회전
    }

    // 재시도 로직 처리
    private void handleRetry() {
        if (retryCount >= maxRetryAttempts) {
            transitionTo(DriveMode.FAILED, "재시도 초과 -> FAILED");
        } else {
            double delay = RETRY_DELAY;
            String reason = "알 수 없음"; // 기본 실패 이유

            try {
                newPath = pathGenerator.buildPathWithConstraints();
                if (newPath == null) {
                    reason = "경로 생성 실패";
                }
            } catch (Exception e) { // 여기서 e를 정의
                reason = e.getMessage() != null ? e.getMessage() : "알 수 없는 오류";
            }

            if (reason.toLowerCase().contains("network")) {
                maxRetryAttempts = 4; // 네트워크 오류는 재시도 횟수를 늘림
                delay = 0.5; // 재시도 간격을 줄임
            } else if (reason.toLowerCase().contains("complex")) {
                maxRetryAttempts = 1; // 경로가 너무 복잡하면 재시도 횟수를 줄임
            }
            
            retryCount++;
            System.out.println("[AdvancedAvoid] Retry attempt " + retryCount + ", reason: " + reason);
            Timer.delay(delay); // 지연 시간을 두어 재시도
            transitionTo(DriveMode.PREPARE_PATH_GENERATION, "재시도 진행");
        }
    }
}