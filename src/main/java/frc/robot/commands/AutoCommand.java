package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathGenerator;
import frc.robot.subsystems.Limelightsub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import org.json.simple.parser.ParseException;
import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoCommand extends Command {
  private final DriveSubsystem drive;
  private final Limelightsub limelight;
  private final DynamicPathGenerator pathGenerator;
  private final double xGoal;
  private final double yGoal;
  
  private boolean finished = false;
  private double startTime = 0.0;
  private static final double BASE_TIMEOUT_SEC = 5.0; // 기본 타임아웃 시간
  private static final double TIMEOUT_PER_METER = 1.0; // 거리당 추가 시간
  private static final double MAX_TIMEOUT_SEC = 30.0; // 최대 타임아웃 시간
  private double dynamicTimeout;
  
  // 최대 재시도 횟수 설정
  private static final int MAX_ATTEMPTS = 3; 
  private int autoAttemptCount = 0;

  public AutoCommand(DriveSubsystem drive, Limelightsub limelight,
                     DynamicPathGenerator pathGenerator,
                     double xGoal, double yGoal) {
    this.drive = drive;
    this.limelight = limelight;
    this.pathGenerator = pathGenerator;
    this.xGoal = xGoal;
    this.yGoal = yGoal;
  }

  @Override
  public void initialize() {
    finished = false;
    autoAttemptCount = 0;
    startTime = Timer.getFPGATimestamp();
    System.out.println("[AutoCommand] 초기화 시작");

    // 시작 위치와 목표 위치 설정
    Translation2d startPos = new Translation2d(7.2, 4.15);
    Translation2d goalPos = new Translation2d(xGoal, yGoal);
    System.out.println("  StartPos = " + startPos + ", GoalPos = " + goalPos);

    // 동적 타임아웃 계산
    double distance = startPos.getDistance(goalPos); 
    dynamicTimeout = Math.min(BASE_TIMEOUT_SEC + (distance * TIMEOUT_PER_METER), MAX_TIMEOUT_SEC);

    System.out.println("[AutoCommand] Dynamic timeout set to: " + dynamicTimeout + " seconds");

    // AD* 경로 생성 시작
    pathGenerator.startPathGeneration(startPos, goalPos);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    if (pathGenerator.isNewPathAvailable()) {
      PathPlannerPath newPath = pathGenerator.buildPathWithConstraints();
      if (newPath == null) {
        if (autoAttemptCount < MAX_ATTEMPTS) {
          autoAttemptCount++;
          System.err.println("[AutoCommand] \u001B[31mPath build failed on attempt " + 
                             autoAttemptCount + " of " + MAX_ATTEMPTS + " -> Retrying\u001B[0m");
          // 재시도: 타임아웃 및 경로 생성 재요청
          startTime = Timer.getFPGATimestamp();
          retryPathGeneration();
        } else {
          System.err.println("[AutoCommand] \u001B[31mPath build failed after " + MAX_ATTEMPTS + " attempts -> finishing\u001B[0m");
          finished = true;
        }
      } else {
        System.out.println("[AutoCommand] \u001B[32mPath build success! Scheduling FollowPath and Avoid\u001B[0m");
        scheduleFollowAndAvoid(newPath);
        finished = true;
      }
    } else if (now - startTime > dynamicTimeout) {
      System.err.println("[AutoCommand] \u001B[33mDynamic Timeout -> finishing\u001B[0m");
      finished = true;
    }
  }

  private void retryPathGeneration() {
    Translation2d startPos = new Translation2d(7.2, 4.15);
    Translation2d goalPos = new Translation2d(xGoal, yGoal);
    pathGenerator.startPathGeneration(startPos, goalPos);
  }

  private void scheduleFollowAndAvoid(PathPlannerPath path) {
    Command followCmd = createFollowPathCommand(path);
    Command avoidCmd = new AdvancedAvoid(
        drive, limelight, pathGenerator,
        new Translation2d(xGoal, yGoal),
        followCmd
    );
    new ParallelCommandGroup(followCmd, avoidCmd).schedule();
  }

  private Command createFollowPathCommand(PathPlannerPath path) {
    return new InstantCommand(() -> {
      Pose2d startPose = path.getStartingHolonomicPose().orElse(new Pose2d());
      drive.resetOdometry(startPose);
    })
    .andThen(buildFollowPath(path))
    .andThen(() -> {
      drive.setModuleStates(
          Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0))
      );
    });
  }

  private Command buildFollowPath(PathPlannerPath path) {
    if (path == null) {
      return new InstantCommand(() -> System.out.println("FollowPathCommand: 경로가 null입니다."));
    }

    Supplier<Pose2d> poseSupplier = drive::getPose;
    Supplier<ChassisSpeeds> speedsSupplier = drive::getChassisSpeeds;
    BiConsumer<ChassisSpeeds, DriveFeedforwards> output = (speeds, ffs) -> {
      var states = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      drive.setModuleStates(states);
    };

    PathFollowingController controller = new PPHolonomicDriveController(
      new PIDConstants(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, Constants.AutoConstants.kDXController),
      new PIDConstants(Constants.AutoConstants.kPYController, Constants.AutoConstants.kIYController, Constants.AutoConstants.kDYController)
    );

    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }
    if (robotConfig == null) {
      return new InstantCommand(() -> System.out.println("RobotConfig is null"));
    }

    BooleanSupplier shouldFlipPath = () -> false;

    return new FollowPathCommand(
        path, poseSupplier, speedsSupplier, output, controller,
        robotConfig, shouldFlipPath, drive
    );
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("[AutoCommand] end(), interrupted=" + interrupted);
  }
}