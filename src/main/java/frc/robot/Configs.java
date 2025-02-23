package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // 주행 모터 설정
      double drivingFactor = ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction;
      double drivingVelFactor = drivingFactor / 60.0;

      drivingConfig.idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(50);
      drivingConfig.encoder.positionConversionFactor(drivingFactor)
                           .velocityConversionFactor(drivingVelFactor);
      drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                               .pid(0.04, 0.0, 0.01) //전히 확 멈춘다면: P를 더 줄이거나(예: 0.02), D를 증가(예: 0.02).
                               .velocityFF(1.0 / ModuleConstants.kDriveWheelFreeSpeedRps)
                               .outputRange(-1, 1);

      // 회전 모터 설정
      double turningFactor = 2.0 * Math.PI;
      double turningVelFactor = turningFactor / 60.0;

      turningConfig.idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(20);
      turningConfig.absoluteEncoder.inverted(true)
                   .positionConversionFactor(turningFactor)
                   .velocityConversionFactor(turningVelFactor);
      turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                              .pid(1.0, 0.0, 0.0)
                              .outputRange(-1, 1)
                              .positionWrappingEnabled(true)
                              .positionWrappingInputRange(0, turningFactor);
    }
  }
}
