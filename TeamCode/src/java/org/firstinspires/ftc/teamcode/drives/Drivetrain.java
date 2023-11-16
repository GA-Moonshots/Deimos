package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.wrappers.IMU;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class Drivetrain {
    // Drivetrain constants
    public static final double ANGLE_TOLERANCE = 1.5; // The angle, in degrees, that is considered "close enough"
    public static final double DISTANCE_TOLERANCE = 1.0; // The distance, in centimeters, that is considered "close enough"
    public static final double MOTOR_MAX_SPEED = 0.9;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 3;
    public static final double APRIL_TAG_DISTANCE_TARGET = 5;
    public static final double APRIL_TAG_PRECISION = 10;
    public static final double APRIL_TAG_MAX_SPEED = 0.3;

    // Drivetrain instance-specific variables
    protected boolean isFieldCentric = true;
    protected Telemetry telemetry;
    protected IMU imu;
    protected Camera camera;
    public Camera getCamera() {
        return camera;
    }

    // Drivetrain instance-specific variables
    protected boolean turningToAngle = false;

    // General Drivetrain subsystems and get methods for them
    public Telemetry getTelemetry() {
        return telemetry;
    }

    public IMU getIMU() {
        return imu;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }
    public boolean getFieldCentric() {
        return isFieldCentric;
    }



    public enum AprilTagToAlign {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, com.qualcomm.robotcore.hardware.IMU.Parameters parameters) {
        this.imu = new IMU(hardwareMap, parameters);
        this.telemetry = telemetry;
        this.camera = new Camera(hardwareMap, telemetry);
    }



    /**
     * @param forward a double to move in the forward direction
     * @param strafe a double to move in the horizontal direction
     * @param turn a double representing the speed to change the heading
     */
    public abstract void drive(double forward, double strafe, double turn);

    /**
     * @brief Non-blocking call to change motor power levels, limited by Drivetrain.MOTOR_MAX_SPEED
     * @param m1 motor power level 1
     * @param m2 motor power level 2
     * @param m3 motor power level 3
     * @param m4 motor power level 4
     */
    protected abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * @brief Blocking call to a robot rotation
    public abstract void drive(double m1, double m2, double m3, double m4);

    /**
     * @param target the absolute angle to move to
     */
    public abstract void turnRobotToAngle(double target);

    /**
     * @brief Blocking imu-adjusted field centric version of turnRobotToAngle
     * @param target The angle to adjust the current angle by
     */
    public void turnRobotByDegree(double target) { turnRobotToAngle(imu.getZAngle() + target); }

    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    /**
     * @brief Blocking, IMU-stabilized call that adjusts the inputs to automatically run
     * @param angle the angle, relative to the field, that the robot needs to move in
     * @param distance the distance the robot needs to move on the specified angle
     */
    public void autoDriveInDirection(double angle, double distance) {
        double xDistance = Math.cos(angle) * distance;
        double yDistance = Math.sin(angle) * distance;

        while(xDistance > DISTANCE_TOLERANCE && yDistance > DISTANCE_TOLERANCE) {
            xDistance -= imu.getXVelocity();
            yDistance -= imu.getYVelocity();

            //drive(Range.clip(xDistance, -1, 1))
            return;
        }
        stop();
    }

    /**
     *
     * @param alignment the AprilTagToAlign
     * @return true if this method can be called again without adjustment
     */
    public boolean alignToAprilTag(AprilTagToAlign alignment) {
        switch (alignment) {
            case LEFT: telemetry.addData("Aligning To", "Left"); break;
            case CENTER: telemetry.addData("Aligning To", "Center"); break;
            case RIGHT: telemetry.addData("Aligning To", "Right"); break;
        }
        // Get AprilTags
        List<AprilTagDetection> detections = camera.getDetections();
        if(detections.size() == 0) {
            telemetry.addData("Detections", "No AprilTags found");
            return false;
        }

        AprilTagDetection activeDetection = null;
        telemetry.addData("Detections", detections.size());
        // IDENTIFY THE INTENDED TAG
        for(AprilTagDetection detection : detections) {
            if(detection.metadata != null)
                switch (alignment) {
                    case LEFT:
                        if (detection.metadata.name.toLowerCase().contains("left")) {
                            activeDetection = detection;
                        }
                        break;
                    case CENTER:
                        if (detection.metadata.name.toLowerCase().contains("center")) {
                            activeDetection = detection;
                        }
                        break;
                    case RIGHT:
                        if (detection.metadata.name.toLowerCase().contains("right")) {
                            activeDetection = detection;
                        }
                        break;
                }
        }
        // IF UNABLE TO FIND INTENDED TAG
        if(activeDetection == null) {
            activeDetection = detections.get(0);
            switch (alignment) {
                case LEFT:
                    drive(0,APRIL_TAG_MAX_SPEED, 0);
                    break;

                case RIGHT:
                    drive(0, -APRIL_TAG_MAX_SPEED, 0 );
                    break;

                case CENTER:
                    if(activeDetection.metadata.name.toLowerCase().contains("left"))
                        drive(0,-APRIL_TAG_MAX_SPEED, 0);

                    else
                        drive(0, APRIL_TAG_MAX_SPEED, 0);
                    break;
            }
            return true;
        }
        telemetry.addData("Tracking XYZ", "(%.2f, %.2f, %.2f)",
                activeDetection.ftcPose.x, activeDetection.ftcPose.y, activeDetection.ftcPose.z);
        telemetry.addData("Tracking YPR", "(%.2f, %.2f, %.2f)",
                activeDetection.ftcPose.yaw, activeDetection.ftcPose.pitch, activeDetection.ftcPose.roll);

        // According to the alignment, we need to find if one of the tags is the correct one,
        // or if we have to adjust to find it.
        double turn = Range.clip((-activeDetection.ftcPose.yaw / APRIL_TAG_PRECISION), -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);
        telemetry.addData("Position", "Finalizing");
        // TODO: If we keep headbutting the board, insert a distance sensor check here and override the below forward calculation
        double forward = 0; //Range.clip((activeDetection.ftcPose.y - APRIL_TAG_DISTANCE_TARGET) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);
        // Reversed since the camera is on the back of the robot
        double strafe = Range.clip((-activeDetection.ftcPose.x) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);

        if (
                Math.abs(forward) <= Constants.INPUT_THRESHOLD &&
                Math.abs(strafe) <= Constants.INPUT_THRESHOLD &&
                Math.abs(turn) <= Constants.INPUT_THRESHOLD
        ) {
            stop();
            telemetry.addData("Movement", "Done");
            return false;
        }

        telemetry.addData("Movement", "(%.2f, %.2f, %.2f)",
                forward, strafe, turn);

        drive(forward, strafe, turn);

        return detections.size() != 0;

    }
}
