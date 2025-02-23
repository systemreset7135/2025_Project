// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ESCmd extends Command {
    private final ElevatorSubsystem m_elevator;

    public ESCmd(ElevatorSubsystem m_elevator) {
        this.m_elevator = m_elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.incrementPressCount();
        // m_elevator.Testvelocitymax(1);
    }

    @Override
    public boolean isFinished() {
        return true; // 작업 후 즉시 종료
    }
}


