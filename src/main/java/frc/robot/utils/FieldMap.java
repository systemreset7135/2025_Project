package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class FieldMap {
    public static final double FIELD_WIDTH = 17.548;
    public static final double FIELD_HEIGHT = 8.052;
    public static final double NODE_SIZE = 0.3;

    private static final boolean[][] GRID = {
        {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true},
        {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true},
        {true,true,true,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true,true,true,true,true},
    };

    public static List<Translation2d> getObstacles() {
        List<Translation2d> obstacles = new ArrayList<>();
        for (int y = 0; y < GRID.length; y++) {
            for (int x = 0; x < GRID[y].length; x++) {
                if (!GRID[y][x]) {
                    obstacles.add(new Translation2d(x * NODE_SIZE, y * NODE_SIZE));
                }
            }
        }
        return obstacles;
    }
}
