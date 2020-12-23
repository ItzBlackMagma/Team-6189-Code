package org.firstinspires.ftc.teamcode;

public class Locations { // returns the locations of certain objects in Inches

    // Utilities and Conversions
    public static final double mmPerInch = 25.4f; // in * mmPerInch = mm   |    mm / mmPerInch = in
    public static final double mmFTCFieldWidth = (12 * 12) * mmPerInch, mmFTCHalfField = mmFTCFieldWidth / 2;
    public static final double inFTCFieldWidth = (12 * 12), inFTCHalfField = inFTCFieldWidth / 2;
    public static final double mmBotWidth = 18 * mmPerInch;

    // X, Y
    public static final double START_STACK[] = { 23.75, 37.0 };
    public static final double TARGET_ZONE_A[] = { 8.25, 60.5 }, TARGET_ZONE_B[] = { 34.125, 37.5 }, TARGET_ZONE_C[] = { 57.872, 60.5 };
    public static final double TARGET_ZONES[][] = { TARGET_ZONE_A, TARGET_ZONE_B, TARGET_ZONE_C }; // groups the zones together (TARGET_ZONES[0][1])
    public static final double BOUNDS[] = { inFTCHalfField, inFTCFieldWidth * 2/3 };

    // X, Y, Z
    public static final double HIGH_GOAL[] = { inFTCHalfField, 33.75, 35.25 };
    public static final double POWER_SHOT_A[] = { inFTCHalfField, 3.75, 33.25 }; // closest to center
    public static final double POWER_SHOT_B[] = { inFTCHalfField, 11.25, 33.25 }; // middle
    public static final double POWER_SHOT_C[] = { inFTCHalfField, 18.75, 33.25 }; // closest to tower goal
    public static final double TARGETS[][] = {HIGH_GOAL, POWER_SHOT_A, POWER_SHOT_B, POWER_SHOT_C}; // groups the targets together (TARGETS[0][1])

    // X
    public static final double LAUNCH_LINE = 9.25;

}
