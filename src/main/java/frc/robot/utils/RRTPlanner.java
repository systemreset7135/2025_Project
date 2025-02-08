package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RRTPlanner {
    private static final double STEP_SIZE = 0.5; // 한 번 확장할 때 이동 거리 (m)
    private static final int MAX_ITERATIONS = 500; // 최대 반복 횟수

    public static List<Pose2d> generateRRTPath(Pose2d start, Pose2d goal, List<Translation2d> obstacles) {
        List<Pose2d> path = new ArrayList<>();
        path.add(start);

        Random rand = new Random();

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double randX = rand.nextDouble() * 17.548; // 필드 크기 고려
            double randY = rand.nextDouble() * 8.052;

            Translation2d randomPoint = new Translation2d(randX, randY);

            Pose2d nearest = findNearestNode(path, randomPoint);
            Translation2d direction = randomPoint.minus(nearest.getTranslation());
            double distance = direction.getNorm();
            Translation2d newPoint = nearest.getTranslation().plus(direction.div(distance).times(STEP_SIZE));

            if (isCollision(newPoint, obstacles)) continue;

            path.add(new Pose2d(newPoint, new Rotation2d()));

            if (newPoint.getDistance(goal.getTranslation()) < STEP_SIZE) {
                path.add(goal);
                break;
            }
        }

        return path;
    }

    private static Pose2d findNearestNode(List<Pose2d> path, Translation2d point) {
        Pose2d nearest = path.get(0);
        double minDistance = Double.MAX_VALUE;

        for (Pose2d node : path) {
            double distance = node.getTranslation().getDistance(point);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = node;
            }
        }

        return nearest;
    }

    private static boolean isCollision(Translation2d point, List<Translation2d> obstacles) {
        for (Translation2d obs : obstacles) {
            if (point.getDistance(obs) < 0.3) return true;
        }
        return false;
    }

    // 📌 🚀 추가된 최적 경로 계산 함수
    public static List<Pose2d> getOptimizedPath(List<Pose2d> path) {
        if (path == null || path.size() < 2) return path; // 경로가 너무 짧으면 그대로 반환

        List<Pose2d> optimizedPath = new ArrayList<>();
        optimizedPath.add(path.get(0)); // 출발점 추가

        for (int i = 1; i < path.size() - 1; i++) {
            Pose2d prev = optimizedPath.get(optimizedPath.size() - 1);
            Pose2d current = path.get(i);
            Pose2d next = path.get(i + 1);

            // ❌ 만약 prev → current → next가 직선이라면 중간점(current) 제거
            if (!isCollinear(prev, current, next)) {
                optimizedPath.add(current);
            }
        }

        optimizedPath.add(path.get(path.size() - 1)); // 목표점 추가
        return optimizedPath;
    }

    // 세 점이 같은 직선 상에 있는지 확인하는 함수
    private static boolean isCollinear(Pose2d p1, Pose2d p2, Pose2d p3) {
        double area = Math.abs((p1.getX() * (p2.getY() - p3.getY()) +
                                p2.getX() * (p3.getY() - p1.getY()) +
                                p3.getX() * (p1.getY() - p2.getY())) / 2.0);
        return area < 0.01; // 삼각형의 면적이 거의 0이면 같은 직선 상에 있음
    }
}
