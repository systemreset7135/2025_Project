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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants;

import frc.robot.commands.AdvancedAvoid;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ESCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathGenerator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.Limelightsub;

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

    private final PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);

    private Command m_cachedAutoCommand = null;
    
    private double autoTargetX;
    private double autoTargetY;

    public RobotContainer() {
        configureButtonBindings();
        

        loadAutoTargets(); // 파일에서 타겟 위치 읽어오기

        // 기본 명령 설정
        // m_elevator.setDefaultCommand(m_elevatorCmd); // 주석 처리된 상태 유지
        m_drive.setDefaultCommand(
            new RunCommand(() -> m_drive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_drive)
        ); 
    }

    private void configureButtonBindings() {
        // 모듈 X 셋 버튼
        new JoystickButton(m_driverController, OIConstants.kDriverSetXIndex)
            .whileTrue(new RunCommand(() -> m_drive.setX(), m_drive));

            JoystickButton elevatorButton = new JoystickButton(m_driverController, OIConstants.kDriverElevatorL1Index);
            elevatorButton.onTrue(new ESCmd(m_elevator));
        }







    // 자율 주행 명령 캐싱 및 상수 설정 메서드
    public Command getAutonomousCommand() {
        if (m_cachedAutoCommand == null) {
            m_cachedAutoCommand = new AutoCommand(m_drive, m_limelight, m_pathGenerator, autoTargetX, autoTargetY);
        }
        return m_cachedAutoCommand;
    }

    // 타겟 위치 동적 설정 메서드
    public void setAutoTarget(double x, double y) {
        this.autoTargetX = x;
        this.autoTargetY = y;
        m_cachedAutoCommand = null; // 새 타겟 설정 시 캐시 초기화
    }

    // 타겟 위치 파일에서 읽어오기
    private void loadAutoTargets() {
        try {
            JSONObject targets = (JSONObject) new JSONParser().parse(new FileReader("targets.json"));
            autoTargetX = ((Number) targets.get("x")).doubleValue();
            autoTargetY = ((Number) targets.get("y")).doubleValue();
            m_cachedAutoCommand = null;
        } catch (IOException | ParseException e) {
            System.err.println("Failed to load auto targets: " + e.getMessage());
            autoTargetX = 1.0; // 기본값 설정
            autoTargetY = 2.0;
        }
    }
}