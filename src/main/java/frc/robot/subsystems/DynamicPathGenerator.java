package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Filesystem; // WPILib
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

// JSON-Simple
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/**
 * DynamicPathGenerator:
 *  1) AD* 알고리즘을 통해 20초 동안 경로(PathPlannerPath)를 생성 대기
 *  2) 경로 생성 성공 시 .path 파일로 내보내기
 *  3) PathPlanner GUI에서 경로 시각화 가능
 */
public class DynamicPathGenerator extends SubsystemBase {
    // 원하는 최대 속도 등 (더 자세한 상수는 Constants에 통합 가능)
    private static final double MAX_SPEED_M_S     = 3.0; 
    private static final double MAX_ACCEL_M_S2    = 3.0; 
    private static final double MAX_ANG_VEL_RAD_S = Math.toRadians(540.0);
    private static final double MAX_ANG_ACC_RAD_S2= Math.toRadians(720.0);

    private static final long PATH_CALCULATION_TIMEOUT = 20; // 타임아웃 시간 (초)

    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private CompletableFuture<PathPlannerPath> pathFuture = null;

    public DynamicPathGenerator() {
        Pathfinder adStar = new LocalADStar();
        Pathfinding.setPathfinder(adStar);
    }

    /** 경로 생성 시작 (비동기) */
    public void startPathGeneration(Translation2d startPos, Translation2d goalPos) {
        pathFuture = CompletableFuture.supplyAsync(() -> {
            try {
                Pathfinding.setStartPosition(startPos);
                Pathfinding.setGoalPosition(goalPos);
                Pathfinding.ensureInitialized();
                
                PathPlannerPath path = buildPathWithConstraints();
                if (path == null) {
                    throw new Exception("Path building failed.");
                }
                return path;
            } catch (Exception e) {
                System.err.println("❌ 경로 생성 시작 중 오류: " + e.getMessage());
                e.printStackTrace();
                return null;
            }
        }, executor).completeOnTimeout(null, PATH_CALCULATION_TIMEOUT, TimeUnit.SECONDS);
    }

    /** Pathfinding이 새 경로를 찾았는지 */
    public boolean isNewPathAvailable() {
        return pathFuture != null && pathFuture.isDone();
    }

    /** Path가 준비되었다면 최종 PathPlannerPath를 빌드 & 반환 */
    public PathPlannerPath buildPathWithConstraints() {
        try {
            // 제약사항
            PathConstraints constraints = new PathConstraints(
                MAX_SPEED_M_S, MAX_ACCEL_M_S2, 
                MAX_ANG_VEL_RAD_S, MAX_ANG_ACC_RAD_S2,
                12.0, false  // nominalVoltage, unlimited
            );
            // 목표 종단 상태(예: 속도=0, 회전=-142.582도)
            GoalEndState goalState = new GoalEndState(0.0, Rotation2d.fromDegrees(-142.582));

            // 빌드
            PathPlannerPath newPath = Pathfinding.getCurrentPath(constraints, goalState);
            if (newPath == null) {
                System.err.println("❌ AD* 경로 빌드 실패! 현재 경로 제약 조건: " + constraints.toString());
                return null;
            } else {
                System.out.println("✅ AD* 경로 빌드 성공! 경로 길이: " + calculatePathLength(newPath));
                savePathAsFile("DynamicPath", newPath);
                return newPath;
            }
        } catch (Exception e) {
            System.err.println("❌ 경로 빌드 중 오류 발생: " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    /** 경로 생성 결과를 가져옵니다. */
    public PathPlannerPath getNewPath() {
        if (pathFuture != null && pathFuture.isDone()) {
            try {
                return pathFuture.get();
            } catch (Exception e) {
                System.err.println("❌ 경로 결과 가져오기 실패: " + e.getMessage());
                e.printStackTrace();
                return null;
            }
        }
        return null;
    }

    private double calculatePathLength(PathPlannerPath path) {
        double length = 0;
        List<Translation2d> points = path.getAllPathPoints().stream().map(point -> point.position).collect(Collectors.toList());
        for (int i = 0; i < points.size() - 1; i++) {
            length += points.get(i).getDistance(points.get(i + 1));
        }
        return length;
    }

    /**
     * PathPlannerPath를 .path 파일(JSON)로 저장하여
     * PathPlanner GUI에서 열어볼 수 있게 하는 유틸 메서드
     * @param pathName 파일 이름(확장자 .path 자동)
     * @param path PathPlannerPath 객체
     */
    private void savePathAsFile(String baseName, PathPlannerPath path) {
        try {
            // 1) JSON 객체 생성
            JSONObject root = new JSONObject();
            root.put("version", "2025.0");
        
            // waypoints 배열 생성
            JSONArray waypointsArr = new JSONArray();
            for (Waypoint w : path.getWaypoints()) {
                JSONObject waypointObj = new JSONObject();
        
                // anchor 좌표 저장
                JSONObject anchorObj = new JSONObject();
                anchorObj.put("x", w.anchor().getX());
                anchorObj.put("y", w.anchor().getY());
                waypointObj.put("anchor", anchorObj);
        
                // prevControl (존재하면 저장)
                if (w.prevControl() != null) {
                    JSONObject prevObj = new JSONObject();
                    prevObj.put("x", w.prevControl().getX());
                    prevObj.put("y", w.prevControl().getY());
                    waypointObj.put("prevControl", prevObj);
                }
        
                // nextControl (존재하면 저장)
                if (w.nextControl() != null) {
                    JSONObject nextObj = new JSONObject();
                    nextObj.put("x", w.nextControl().getX());
                    nextObj.put("y", w.nextControl().getY());
                    waypointObj.put("nextControl", nextObj);
                }
        
                waypointsArr.add(waypointObj);
            }
            root.put("waypoints", waypointsArr);
        
            // 기타 경로 관련 정보 저장 (빈 배열 또는 기본값)
            root.put("rotationTargets", new JSONArray());
            root.put("constraintZones", new JSONArray());
            root.put("pointTowardsZones", new JSONArray());
            root.put("eventMarkers", new JSONArray());
        
            // 글로벌 제약 조건 저장
            JSONObject gc = new JSONObject();
            gc.put("maxVelocity", path.getGlobalConstraints().maxVelocityMPS());
            gc.put("maxAcceleration", path.getGlobalConstraints().maxAccelerationMPSSq());
            gc.put("maxAngularVelocity", Math.toDegrees(path.getGlobalConstraints().maxAngularVelocityRadPerSec()));
            gc.put("maxAngularAcceleration", Math.toDegrees(path.getGlobalConstraints().maxAngularAccelerationRadPerSecSq()));
            gc.put("nominalVoltage", path.getGlobalConstraints().nominalVoltageVolts());
            gc.put("unlimited", path.getGlobalConstraints().unlimited());
            root.put("globalConstraints", gc);
        
            // 목표 종단 상태 저장
            JSONObject goalObj = new JSONObject();
            goalObj.put("velocity", path.getGoalEndState().velocityMPS());
            goalObj.put("rotation", Math.toDegrees(path.getGoalEndState().rotation().getDegrees()));
            root.put("goalEndState", goalObj);
        
            // 추가 정보 설정
            root.put("reversed", false);
            root.put("folder", null);
        
            JSONObject idealStartObj = new JSONObject();
            idealStartObj.put("velocity", 0.0);
            idealStartObj.put("rotation", 0.0);
            root.put("idealStartingState", idealStartObj);
        
            root.put("useDefaultConstraints", false);
        
            // 2) JSON 문자열 생성
            String jsonString = root.toJSONString();
        
            // 3) 파일 경로 지정 및 파일 이름 자동 증가 처리
            File deployDir = Filesystem.getDeployDirectory();
            File folder = new File(deployDir, "pathplanner/paths/");
            if (!folder.exists()) {
                folder.mkdirs();
            }
            
            // 기본 이름과 숫자를 조합하여 파일 이름 결정
            int index = 1;
            File pathFile = new File(folder, baseName + index + ".path");
            while (pathFile.exists()) {
                index++;
                pathFile = new File(folder, baseName + index + ".path");
            }
        
            // 4) 파일로 저장
            try (FileWriter writer = new FileWriter(pathFile)) {
                writer.write(jsonString);
                System.out.println("✅ .path 파일 저장 완료: " + pathFile.getAbsolutePath());
            }
        } catch (IOException e) {
            System.err.println("❌ 파일 저장 중 오류: " + e.getMessage());
            e.printStackTrace();
        }
    }
}