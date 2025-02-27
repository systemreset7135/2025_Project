package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.commands.AdvancedAvoid;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.MEC;
import frc.robot.commands.SEC;
import frc.robot.commands.ShootingCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathGenerator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.Limelightsub;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.SAutocmd;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileReader;
import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    private final GyroSubsystem m_gyro = new GyroSubsystem();
    private final DriveSubsystem m_drive = new DriveSubsystem(m_gyro);
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final DynamicPathGenerator m_pathGenerator = new DynamicPathGenerator();
    private final Limelightsub m_limelight = new Limelightsub();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
    private Command m_cachedAutoCommand = null;
    private double autoTargetX;
    private double autoTargetY;
    private boolean isAutoMode = true; // 엘리베이터 모드: Setpoint 모드 기본 (true = SEC, false = MEC)
    private int setpointIndex = 0; // Setpoint 순환 인덱스

    public RobotContainer() {
        configureButtonBindings();
        loadAutoTargets();

        // 기본 명령 설정
        m_drive.setDefaultCommand(
            new RunCommand(() -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive)
        );
        m_elevator.setDefaultCommand(new MEC(m_elevator, m_driverController));
        m_shooter.setDefaultCommand(new ShootingCmd(m_shooter, "stop"));
        
    }

    private void configureButtonBindings() {
        // 모듈 X 셋 버튼
        new JoystickButton(m_driverController, OIConstants.kDriverSetXIndex)
            .whileTrue(new RunCommand(() -> m_drive.setX(), m_drive));

        // 모드 전환 및 Setpoint 순환 버튼 (버튼 14 - 옵션 버튼)
        new JoystickButton(m_driverController, OIConstants.kDriverMode) // 버튼 14
            .onTrue(new InstantCommand(() -> DriveMode()));

        new JoystickButton(m_driverController, OIConstants.kDriverShooterIntakeIndex).whileTrue(new ShootingCmd(m_shooter, "in"));
        new JoystickButton(m_driverController, OIConstants.kDriverShooterShootIndex).whileTrue(new ShootingCmd(m_shooter, "out"));

        Trigger povUp = new Trigger(() -> m_driverController.getPOV() == 0);
        povUp.whileTrue(new SEC(m_elevator, ElevatorConstants.kSetpoints[2]));

        // POV 下（180°）对应 kSetpoints[1]
        Trigger povDown = new Trigger(() -> m_driverController.getPOV() == 180);
        povDown.whileTrue(new SEC(m_elevator, ElevatorConstants.kSetpoints[0]));

        // POV 右（90°）对应 kSetpoints[2]
        Trigger povRight = new Trigger(() -> m_driverController.getPOV() == 90);
        povRight.whileTrue(new SEC(m_elevator, ElevatorConstants.kSetpoints[3]));

        // POV 左（270°）对应 kSetpoints[3]
        Trigger povLeft = new Trigger(() -> m_driverController.getPOV() == 270);
        povLeft.whileTrue(new SEC(m_elevator, ElevatorConstants.kSetpoints[1]));

        new JoystickButton(m_driverController, OIConstants.kDriverResetElevatorIndex).whileTrue(new RunCommand(() -> m_drive.resetEncoders(), m_elevator));
        // Setpoint 모드에서 순환 실행 (버튼 14를 누를 때마다 순환)
        // new JoystickButton(m_driverController, OIConstants.kDriverElevatorL1Index)
        // .onTrue(new InstantCommand(() -> {
        //     if (isAutoMode) { // Setpoint 모드일 때만 순환
        //         cycleSetpoint();
        //     }
        // }));
    }

    // 모드 전환 메서드 (기존 DriveMode 유지)
    private void DriveMode() {
        isAutoMode = !isAutoMode; // Setpoint 모드와 리모컨 모드 전환
        System.out.println("Mode: " + (isAutoMode ? "tele-Auto Mode" : "tele-tele 모드"));
    }

    // Setpoint 순환 메서드
    private void cycleSetpoint() {
        setpointIndex = (setpointIndex + 1) % 4; // 0, 1, 2, 3을 순환
        double targetSetpoint = ElevatorConstants.kSetpoints[setpointIndex];
        System.out.println("elevator" + targetSetpoint + "inches");
        new SEC(m_elevator, targetSetpoint).schedule(); // 해당 Setpoint로 이동
    }

    // 모드에 따른 명령 반환 (기존 getmodecommand 유지)
    public Command getmodecommand(double setpoint) {
        if (isAutoMode) {
            return new SEC(m_elevator, setpoint);
        } else {
            return new MEC(m_elevator, m_driverController);
        }
    }

    // 자율 주행 명령 캐싱 및 상수 설정 메서드
    public Command getAutonomousCommand() {
        m_gyro.zeroHeading(); // 자이로 초기화

        // 1번 경로로 이동
        Command moveToPath1 = new SAutocmd(m_drive, "C-1");

        // 5초 대기
        Command elevatorToSetpoint = new SEC(m_elevator, ElevatorConstants.kSetpoints[3]);

        Command wait3Seconds = new WaitCommand(3.0);

        // station1 경로로 이동
        Command moveToStation1 = new SAutocmd(m_drive, "STATION 1");

        // 순차적으로 실행
        return new SequentialCommandGroup(
            moveToPath1,
            
            wait3Seconds,
            moveToStation1
        );
    }

    // 타겟 위치 동적 설정 메서드
    public void setAutoTarget(double x, double y) {
        this.autoTargetX = x;
        this.autoTargetY = y;
        m_cachedAutoCommand = null; // 새 타겟 설정 시 캐시 초기화
    }

    // 타겟 위치 파일에서 읽어오기
    private void loadAutoTargets() {
        // targets.json 파일을 무시하고 항상 기본값 사용
        autoTargetX = 2; // 기본값 설정
        autoTargetY = 7;
        m_cachedAutoCommand = null; // 캐시 초기화
        System.out.println("목표 위치를 기본값으로 설정 - X: " + autoTargetX + "m, Y: " + autoTargetY + "m");
    }
}