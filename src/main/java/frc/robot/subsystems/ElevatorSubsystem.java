package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor1;
    private final SparkMax elevatorMotor2;
    private final Encoder encoder = new Encoder(0, 1);
    
    private final PIDController pid = new PIDController(0.1, 0, 0);
    
    private double setpoint = 0.0;
    private static final double MAX_HEIGHT = 35; // inch
    private static final double MIN_HEIGHT = -10; // inch
    private int pressCount = 0;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 5);
    private TrapezoidProfile.State goalposition = new TrapezoidProfile.State(0.0, 0.0);

    private double kG = 0.2;
    private double kV = 0.01;

    private boolean isControlActive = false; 
    private static final double TOLERANCE = 0.5;

    // Encoder와 모터 설정
    private static final double DISTANCE_PER_PULSE = 0.000480315; // inch/pulse
    private static final int PPR = 8192; // Encoder 사양

    public ElevatorSubsystem() {
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE); 
        encoder.reset(); 
        
        elevatorMotor1 = new SparkMax(ElevatorConstants.Elevator1CanId, MotorType.kBrushed);
        elevatorMotor2 = new SparkMax(ElevatorConstants.Elevator2CanId, MotorType.kBrushed);
       
        SmartDashboard.putNumber("Elevator/P Gain", 0.1);
        SmartDashboard.putNumber("Elevator/I Gain", 0.0);
        SmartDashboard.putNumber("Elevator/D Gain", 0.0);
        SmartDashboard.putNumber("Elevator/Setpoint", 0.0);
        SmartDashboard.putNumber("Elevator/kG", kG);
        SmartDashboard.putNumber("Elevator/kV", kV);
    }

    @Override
    public void periodic() {
        double currentHeight = encoder.getDistance();
        double EncoderRate = encoder.getRate();

        if (isControlActive){

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(currentHeight, EncoderRate);
        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, currentState, goalposition);//0.02초마다 어디 쯤 가있을시를 계산 하는거야 

        double targetPosition = nextSetpoint.position;//0.02초 동안 올라갈 수 있는 거리
        double targetVelocity = nextSetpoint.velocity;//0.02초 동안 올라갈 수 있는 속도

        double motorCurrent = elevatorMotor1.getOutputCurrent();
        kG = 0.2 + (motorCurrent / 10.0) * 0.1 + (currentHeight / MAX_HEIGHT) * 0.05;
        kV = 0.01 + Math.abs(targetVelocity) * 0.0003937;
        
        double Orioutput = pid.calculate(currentHeight, targetPosition) + kV * targetVelocity + kG;
        double realoutput = MathUtil.clamp(Orioutput, -0.5, 0.5);

        if (currentHeight >= MAX_HEIGHT && realoutput > 0) {
            realoutput = 0; 
        } else if (currentHeight <= MIN_HEIGHT && realoutput < 0) {
            realoutput = 0;
        }

        elevatorMotor1.set(realoutput);
        elevatorMotor2.set(realoutput);

        // 실제 회전 수 계산 (방향 구분)
        double actualPulses = encoder.get();
        double actualRotations = actualPulses / PPR;

        // SmartDashboard에 추가 정보 표시
        SmartDashboard.putNumber("Elevator/Actual Rotations", actualRotations); // 양수: 위로, 음수: 아래로
        SmartDashboard.putNumber("Elevator/Speed (inch/s)", EncoderRate); // 속도 표시

        Debug(currentHeight, realoutput, Orioutput, EncoderRate, targetPosition, targetVelocity, motorCurrent);
    }else {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0); 
    }
    }

    public double getHeight() {
        return encoder.getDistance();
    }

    public void incrementPressCount() {
        pressCount++;
        double targetHeight;
        switch (pressCount % 3) {
            case 1: targetHeight = 33.0; break; // inch
            case 2: targetHeight = 20.0; break; // inch
            case 0: targetHeight = 0.0; break; // inch
            default: targetHeight = 0.0;
        }
        setHeight(targetHeight);
        isControlActive = true;
        SmartDashboard.putNumber("Elevator/Setpoint (inch)", targetHeight);
        calculateTargetRotations(targetHeight); // 목표 회전 수 계산
        System.out.println("ESCmd initialized, pressCount: " + pressCount + ", targetHeight: " + targetHeight + " inch");
    }

    public void setHeight(double height) {
        setpoint = MathUtil.clamp(height, MIN_HEIGHT, MAX_HEIGHT);
        goalposition = new TrapezoidProfile.State(setpoint, 0.0);
    }

    public double getsetpointvisual() {
        return setpoint;
    }

    private void Debug(double currentHeight, double realoutput, double Orioutput, double EncoderRate, double targetPosition, double targetVelocity, double motorCurrent) {
        
        SmartDashboard.putNumber("Elevator/Current Height (inch)", currentHeight);
        SmartDashboard.putNumber("Elevator/original PID Output", Orioutput);
        SmartDashboard.putNumber("Elevator/futurePosition", targetPosition);
        SmartDashboard.putNumber("Elevator/futureVelocity", targetVelocity);
        SmartDashboard.putNumber("Elevator/Motor Current (A)", motorCurrent);
        SmartDashboard.putNumber("Elevator/realoutput", realoutput);

        double p = SmartDashboard.getNumber("Elevator/P Gain", pid.getP());
        double i = SmartDashboard.getNumber("Elevator/I Gain", pid.getI());
        double d = SmartDashboard.getNumber("Elevator/D Gain", pid.getD());

        if (p != pid.getP()) {
            System.out.println("New input P Gain: " + p);
            pid.setP(p);
        }
        if (i != pid.getI()) {
            System.out.println("New input I Gain: " + i);
            pid.setI(i);
        }
        if (d != pid.getD()) {
            System.out.println("New input D Gain: " + d);
            pid.setD(d);
        }

        double newSetpoint = SmartDashboard.getNumber("Elevator/Setpoint", setpoint);
        if (newSetpoint != setpoint) {
            System.out.println("New input Setpoint: " + newSetpoint);
            setHeight(newSetpoint);
        }
    }

    private void calculateTargetRotations(double targetHeight) {
        double targetPulses = targetHeight / DISTANCE_PER_PULSE;
        double targetRotations = targetPulses / PPR;
        SmartDashboard.putNumber("Elevator/Target Rotations", targetRotations);
        System.out.println("Target Rotations: " + targetRotations);
    }//모터회전수 계산 하는 로직
}

// public void Testvelocitymax (double output){
//     double maxVelocity = 0.0;
//     double maxAcceleration = 0.0;
//     double prevVelocity = 0.0;
//     double currentVelocity = 0.0;
//     double acceleration = 0.0;
//     double startTime = Timer.getFPGATimestamp();
//     double lastTime = startTime;
    
//     // 모터 최대 출력 실행
//     elevatorMotor1.set(output);
//     elevatorMotor2.set(output);
    
//     // 1초 동안 최대 속도와 가속도 측정
//     while (Timer.getFPGATimestamp() - startTime < 2.0) {
//         double currentTime = Timer.getFPGATimestamp();
//         currentVelocity = encoder.getRate();  // cm/s 단위로 가정
        
//         // 최대 속도 갱신
//         if (currentVelocity > maxVelocity) {
//             maxVelocity = currentVelocity;
//         }
        
//         // 시간 간격이 충분하면 가속도 계산: (현재 속도 - 이전 속도) / (현재 시간 - 이전 시간)
//         double dt = currentTime - lastTime;
//         if (dt > 0) {
//             acceleration = (currentVelocity - prevVelocity) / dt;  // cm/s² 단위
//             if (acceleration > maxAcceleration) {
//                 maxAcceleration = acceleration;
//             }
//         }
        
//         prevVelocity = currentVelocity;
//         lastTime = currentTime;
        
//         Timer.delay(0.01);  // 10ms 대기
//     }
    
//     // 모터 정지
//     elevatorMotor1.set(0);
//     elevatorMotor2.set(0);
    
//     // 결과 출력: SmartDashboard와 콘솔에 표시
//     System.out.println("Max Velocity: " + maxVelocity + " cm/s");
//     System.out.println("Max Acceleration: " + maxAcceleration + " cm/s²");
    
// }


  
       
    // public void move(String direction){
    //     if(direction == "in"){
    //         elevatorMotor1.set(-0.5);
    //         elevatorMotor2.set(-0.5);
    //     }
    //     else if(direction == "out"){
    //         elevatorMotor1.set(0.5);
    //         elevatorMotor2.set(0.5);

    //     }
    //     else{
    //         elevatorMotor1.set(0);
    //         elevatorMotor2.set(0);
    //     }
    // }



// 2. 어떤 상황에서 어떻게 조정하면 좋을까?
// 2.1 PID 튜닝 시나리오
// P(비례제어) 값이 너무 작으면

// 엘리베이터가 목표점에 거의 못 가거나, 가다가 힘이 부족해 느리게 올라가거나 멈출 수 있어요.
// 이때 P를 조금씩 올려보면, 목표를 향해 더 힘차게 움직임.
// 하지만 P가 너무 크면, 과하게 출렁거리거나(진동, overshoot) 멈추지 못하고 왔다 갔다 할 수 있어요.
// I(적분제어) 값

// P만으로는 목표점 근처에서 작은 오차를 못 없앤다면, I값을 조금 주면 도움이 됨.
// 다만 너무 크면 오히려 흔들릴 수 있으니 조심스럽게 소량씩.
// D(미분제어) 값

// 목표 지점 근처에서 과도하게 떨거나 진동이 있을 때, D값을 살짝 주면 진동을 줄여줌.
// 너무 크게 주면, 엘리베이터가 급격히 멈추려다 또 이상한 진동을 유발할 수도 있음.
// > 실제 조절 순서(가이드라인)
// P부터 조금씩 키워본다. (엘리베이터가 어느 정도 목표점까지 갈 수 있을 만큼)
// 너무 튀거나 심하게 흔들리면 P를 좀 낮추거나, D를 살짝 넣어서 안정화 시킴.
// 목표점 근처에서 자꾸 “조금 모자라거나” “조금씩 밀리는” 오차가 있다면 I를 아주 조그맣게 추가.