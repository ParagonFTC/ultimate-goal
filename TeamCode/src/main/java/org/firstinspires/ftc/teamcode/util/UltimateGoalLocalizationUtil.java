package org.firstinspires.ftc.teamcode.util;

public class UltimateGoalLocalizationUtil {
    public static final double BLUE_GOAL_X = 72;
    public static final double BLUE_GOAL_Y = 36;

    public static double distanceToTowerGoal(double robotX, double robotY) {
        double dx = robotX - BLUE_GOAL_X;
        double dy = robotY - BLUE_GOAL_Y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static double angleToTowerGoal(double robotX, double robotY) {
        double dx = robotX - BLUE_GOAL_X;
        double dy = robotY - BLUE_GOAL_Y;
        return Math.atan2(-dy,-dx);
    }
}
