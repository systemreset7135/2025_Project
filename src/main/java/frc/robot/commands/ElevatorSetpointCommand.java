// ElevatorSetpointCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSetpointCommand extends Command {
    private final ElevatorSubsystem elevator;
    // pressCount는 최대 3까지만 의미가 있음.
    private int pressCount = 0;
    private final Timer timer = new Timer();
    private static final double PRESS_TIMEOUT = 0.5; // 버튼 누름 후 0.5초 동안 추가 입력 없으면 처리
    // 마지막으로 설정한 목표 높이 (초기값: 0인치)
    private double lastSetTarget = ElevatorConstants.kSetpoints[0];

    public ElevatorSetpointCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    // 이 메소드는 버튼이 눌릴 때마다 호출됩니다.
    public void buttonPressed() {
        // pressCount는 최대 3으로 제한 (3번 이상 누르면 3으로 처리)
        if (pressCount < 3) {
            pressCount++;
        }
        timer.reset();
        timer.start();
    }
    
    

    @Override
    public void initialize() {
        pressCount = 0;
        timer.reset();
        timer.start();
        System.out.println("ElevatorSetpointCommand initialized.");
    }
    

    @Override
    public void execute() {
        // 타이머가 PRESS_TIMEOUT 이상 경과하고, 버튼 입력이 있었으면 처리
        if (timer.hasElapsed(PRESS_TIMEOUT) && pressCount > 0) {
            double candidateTarget;
            if (pressCount == 1) {
                candidateTarget = ElevatorConstants.kSetpoints[1]; // 예: 10 inches
            } else if (pressCount == 2) {
                candidateTarget = ElevatorConstants.kSetpoints[2]; // 예: 20 inches
            } else { // pressCount >= 3
                candidateTarget = ElevatorConstants.kSetpoints[3]; // 예: 48 inches
            }
            // 최종 후보에 대해서 단 한 번만 메시지를 출력합니다.
            System.out.println("Elevator button pressed. Count: " + pressCount 
                + " -> Candidate target: " + candidateTarget + " inches");
            
            // 만약 현재 설정된 목표와 후보가 다르면, 목표 높이를 업데이트
            if (candidateTarget != lastSetTarget) {
                System.out.println("Timeout reached. Setting elevator to target height: " + candidateTarget + " inches");
                elevator.setTargetHeight(candidateTarget);
                lastSetTarget = candidateTarget;
            }
            // 버튼 입력 처리가 완료되었으므로 pressCount를 0으로 리셋합니다.
            pressCount = 0;
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Default Command로 계속 실행
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("ElevatorSetpointCommand ended. Final press count was: " + pressCount);
    }
}