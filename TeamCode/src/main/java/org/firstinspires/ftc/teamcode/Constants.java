package org.firstinspires.ftc.teamcode;

public class Constants {
    // ----ARM VARIABLES----
    // ----ARM VARIABLES----
    public static final int ARM_UP_POSITION = -1400;
    public static final int ARM_DOWN_POSITION = 0;
    public static final double ARM_MOTOR_STRENGTH = 0.9;
    public static final double WRIST_ON_WALL =  0.6;
    public static final double CLAW_OPEN_POS = 0.5;
    public static final double CLAW_CLOSED_POS = 0.9;
    public static final double WRIST_INC = 0.01;
    public static final double WRIST_MAX = 0.8;
    public static final double WRIST_ON_GROUND = 0.36;
    public static final double WRIST_MIN = 0.0;
    public static final double ROLL_MAX = 0.72;
    public static final double ROLL_MIN = 0;
    public static final double ROLL_INC = 0.03;

    // -------DRIVE--------
    // -------DRIVE--------
    public static final double ANGLE_TOLERANCE = 1.5; // The angle, in degrees, that is considered "close enough"
    public static final double DISTANCE_TOLERANCE = 1.0; // The distance, in centimeters, that is considered "close enough"
    public static final double MOTOR_MAX_SPEED = 0.9;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 3;
    public static final double APRIL_TAG_DISTANCE_TARGET = 5;
    public static final double APRIL_TAG_PRECISION = 10;
    public static final double APRIL_TAG_MAX_SPEED = 0.3;

    // Generalized minimum input for the robot to respond to controller input
    public static final double INPUT_THRESHOLD = 0.1;
    // Generalized minimum angle difference for the robot to respond to an autonomous movement command
    public static final double ANGLE_THRESHOLD = 3.0;
    // Generalized minimum position difference for the robot to respond to an autonomous movement command
    public static final double DISTANCE_THRESHOLD = 1.0;

    // ----- HARDWARE MAP NAMES ------
    // ----- HARDWARE MAP NAMES ------
    public static final String IMU_NAME = "imu";
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";
    public static final String REAR_DIST_NAME = "rear_distance";
    public static final String RIGHT_DIST_NAME = "right_distance";
    public static final String LEFT_DIST_NAME = "left_distance";
    public static final String ARM_NAME = "arm";
    public static final String WRIST_SERVO_NAME = "wrist";
    public static final String OPEN_SERVO_NAME = "open";
    public static final String ROLL_SERVO_NAME = "roll";
    public static final String RELEASE_SERVO_NAME = "releasePin";
    public static final String WEBCAM_NAME = "Webcam 1";
}
