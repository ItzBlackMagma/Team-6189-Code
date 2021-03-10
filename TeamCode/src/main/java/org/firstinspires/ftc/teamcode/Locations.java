package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.odometry.Waypoint;

public class Locations {
    // All measurements are in inches. Y is from the back wall to the goals. X is from the side wall to the other wall
    public static final double LAUNCH_LINE = 81;

    public static final double WOBBLE_ZONE_A = 81; // distance from back wall to the middle of the wobble zone A
    public static final double WOBBLE_ZONE_B = 107; // distance from back wall to the middle of the wobble zone B
    public static final double WOBBLE_ZONE_B_OFFSET = 35; // distance from the side wall to the middle of the wobble zone B
    public static final double WOBBLE_ZONE_C = 130; // distance from back wall to the middle of the wobble zone C

    public static final double LINE_1 = 22; // distance from the side wall to the starter line closest to it
    public static final double LINE_2 = 45.5; // distance from the side wall to the starter line farthest from it

    public static final double STARTER_STACK_Y = 48.25;
    public static final double STARTER_STACK_X = 35.14;

    public static final double GOAL_TO_STACK = 93;
    public static final double PSA_TO_STACK = 94;
    public static final double PSB_TO_STACK = 95.5;
    public static final double PSC_TO_STACK = 97.5;
    public static final double TARGETS[] = {GOAL_TO_STACK, PSA_TO_STACK, PSB_TO_STACK, PSC_TO_STACK};

    public static final double ANGLE_TO_PSA = -Math.asin((4.25) / PSA_TO_STACK);
    public static final double ANGLE_TO_PSB = -Math.asin((7.5 + 4.25) / PSB_TO_STACK);
    public static final double ANGLE_TO_PSC = -Math.asin((7.5 * 2 + 4.25) / PSC_TO_STACK);

    public static final double robotLaunchHeight = 5, HighGoalHeight = 36, PowerShotHeight = 32;

    public static final double fieldLength = 144;

    public static double distanceBetweenPoints(double start, double finish){
        return  finish - start;
    }

}
