package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

@Config
public class Constants {
    public enum OpModeType {
        AUTO,
        TELEOP
    }
    public enum AllianceColor {
        RED,
        BLUE
    }
    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static Pose2d END_POSE = new Pose2d();

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_VELOCITY = 7.75 * 12; // Inches/second
    public static double PINPOINT_TELEOP_POLLING_RATE = 20; // Hertz
    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0, 0, 0, 0);
    public static double FR_ENCODER_OFFSET = 0.733; // Radians
    public static double FL_ENCODER_OFFSET = 5.018; // Radians
    public static double BL_ENCODER_OFFSET = 3.234; // Radians
    public static double BR_ENCODER_OFFSET = 1.24; // Radians

}
