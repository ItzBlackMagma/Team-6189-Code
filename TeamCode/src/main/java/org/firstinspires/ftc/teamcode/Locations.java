package org.firstinspires.ftc.teamcode;

public class Locations { // returns the locations of certain objects in Inches

    // Utilities and Conversions
    public static final double mmPerInch = 25.4f; // in * mmPerInch = mm   |    mm / mmPerInch = in
    public static final double mmFTCFieldWidth = (12 * 12) * mmPerInch, mmFTCHalfField = mmFTCFieldWidth / 2;
    public static final double mmBotWidth = 18 * mmPerInch;

    // X, Y
    public static final double START_STACK[] = { 23.75, 37.0 };
    public static final double TARGET_ZONE_A[] = { 8.25, 60.5 }, TARGET_ZONE_B[] = { 34.125, 37.5 }, TARGET_ZONE_C[] = { 57.872, 60.5 };
    public static final double[] TARGET_ZONES[] = { TARGET_ZONE_A, TARGET_ZONE_B, TARGET_ZONE_C }; // groups the zones together (TARGET_ZONES[0][1])
    public static final double BOUNDS[] = { mmFTCFieldWidth / mmPerInch, (mmFTCFieldWidth * 2/3) / mmPerInch };

    // X, Y, Z
    public static final double HIGH_GOAL[] = { mmFTCFieldWidth / mmPerInch, mmFTCHalfField / mmPerInch, 35.25 };
    public static final double POWER_SHOT_A[] = { mmFTCFieldWidth / mmPerInch, 0, 33.25 };
    public static final double POWER_SHOT_B[] = { mmFTCFieldWidth / mmPerInch, 0, 0 };
    public static final double POWER_SHOT_C[] = { mmFTCFieldWidth / mmPerInch, 0, 0 };

}
