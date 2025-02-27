// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class GyroSubsystem extends SubsystemBase {
    // Pigeon2 자이로 센서
    private final Pigeon2 m_gyro;
    private boolean isGyroConnected;
    private double yawoffset = 0.0;

    /** GyroSubsystem 생성자 */
    public GyroSubsystem() {
        m_gyro = new Pigeon2(DriveConstants.kPigeonCanId); 
        isGyroConnected = checkConnection();
        if(!isGyroConnected) {
            System.out.println("[GyroSubsystem] no Gyro connection");
        }

    }

    @Override
    public void periodic() {
        // SmartDashboard에 자이로 데이터 출력 (선택 사항)
        SmartDashboard.putNumber("Pigeon Yaw", getYaw());
    }
    public void setYaw(double yawDegrees) {
        m_gyro.setYaw(yawDegrees);
    }

    
    private boolean checkConnection() {
        m_gyro.getVersion();
        return BaseStatusSignal.isAllGood();
    }
    

    public boolean isGyroConnected() {
        return isGyroConnected;
    }


    /**
     * 자이로 센서를 재설정하여 현재 방향을 0으로 만듭니다.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * 로봇의 현재 방향(heading)을 반환합니다.
     * @return 방향(도 단위), -180 ~ 180도
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
    }

    /**
     * 로봇의 회전 속도(turn rate)를 반환합니다.
     * @return 회전 속도(도/초)
     */
    public double getTurnRate() {
        return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * 자이로의 현재 요각(yaw)을 반환합니다.
     * @return 요각(도 단위)
     */
    public double getYaw() {
        return m_gyro.getYaw().getValueAsDouble();
    }

    /**
     * 자이로의 Rotation2d 객체를 반환합니다.
     * @return Rotation2d 형식의 방향
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }
}