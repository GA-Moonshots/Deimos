package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.IMU;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class MecanumDrive {
    // MOTORS
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    // SENSORS
    public DistanceSensor rearDistance;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;
    public Camera camera;
    public IMU imu;
    public Telemetry telemetry;

    // INSTANCE VARIABLES
    private LinearOpMode opMode;
    private double gyroTarget = 0.0d;
    private double lastAprilTagYaw = 0.0d;
    private double lastAprilTagStrafe = 0.0d;
    private double fieldCentricTarget = 0.0d;
    public boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;
    public enum AprilTagToAlign {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    /**
     * This constructor takes a reference to the aggregate object that owns it. It's a controversial
     * design decision and one that can be avoided with a command scheduler
     * @param opMode
     */
    public MecanumDrive(LinearOpMode opMode) {

        this.imu = new IMU(opMode.hardwareMap);
        this.telemetry = opMode.telemetry;
        this.camera = new Camera(opMode.hardwareMap, opMode.telemetry);
        this.opMode = opMode;


        leftFront = opMode.hardwareMap.get(DcMotor.class, Constants.LEFT_FRONT_NAME);
        rightFront = opMode.hardwareMap.get(DcMotor.class, Constants.RIGHT_FRONT_NAME);
        leftBack = opMode.hardwareMap.get(DcMotor.class, Constants.LEFT_BACK_NAME);
        rightBack = opMode.hardwareMap.get(DcMotor.class, Constants.RIGHT_BACK_NAME);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearDistance = new DistanceSensor(opMode.hardwareMap, Constants.REAR_DIST_NAME);
        rightDistance = new DistanceSensor(opMode.hardwareMap, Constants.RIGHT_DIST_NAME);
        leftDistance = new DistanceSensor(opMode.hardwareMap, Constants.LEFT_DIST_NAME);
        this.camera = new Camera(opMode.hardwareMap, telemetry);
        fieldCentricTarget = imu.getZAngle();
    }

    // ---- CORE DRIVE COMMANDS ----
    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    public void drive(double forward, double strafe, double turn) {

        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double diff = fieldCentricTarget - imu.getZAngle();
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(diff)) - strafe * Math.sin(Math.toRadians(diff));
            strafe = temp * Math.sin(Math.toRadians(diff)) + strafe * Math.cos(Math.toRadians(diff));
            if(telemetry != null)
                telemetry.addData("Mode", "Field Centric");
        } else if(telemetry != null)
            telemetry.addData("Mode", "Robot Centric");

        isGyroLocked = turn <= Constants.INPUT_THRESHOLD;
        if(isGyroLocked && !isTargetSet) {
            gyroTarget = imu.getYAngle();
            isTargetSet = true;
        } else if(!isGyroLocked) {
            isTargetSet = false;
        }

        // I'm tired of figuring out the input problems so the inputs are still in flight stick mode
        // Meaning forward is reversed
        // The boost values should match the turn
        // Since the drive is a diamond wheel pattern instead of an X, it reverses the strafe.
        double leftFrontPower = -forward +strafe + turn;
        double rightFrontPower = forward + strafe + turn;
        double leftBackPower = -forward - strafe + turn;
        double rightBackPower = forward - strafe + turn;

        double powerScale = Constants.MOTOR_MAX_SPEED * Math.max(1,
                Math.max(
                        Math.max(
                                Math.abs(leftFrontPower),
                                Math.abs(leftBackPower)
                        ),
                        Math.max(
                                Math.abs(rightFrontPower),
                                Math.abs(rightBackPower)
                        )
                )
        );

        leftFrontPower /= powerScale;
        leftBackPower /= powerScale;
        rightBackPower /= powerScale;
        rightFrontPower /= powerScale;


        if(telemetry != null)
            telemetry.addData("Motors", "(%.2f, %.2f, %.2f, %.2f)",
                    leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        drive(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );
    }

    protected void drive(double m1, double m2, double m3, double m4) {
        leftFront.setPower(Range.clip(m1, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
    }

    // ----- FWD AND BACKWARD -----

    /**
     * Drives away from wall until rear sensor reaches the requested distance or until 3 seconds
     * have passed
     * @param distance requested distance in inches
     */
    public void fwdFromWall(double distance){
        ElapsedTime rt = new ElapsedTime();
        while(rearDistance.getDistance() <= distance && rt.seconds() <= 3
                && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance());
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
            drive(-0.3, 0.0, 0.0);
        }
        stop();
    }

    /**
     * @param time seconds to drive
     * @param str negative is fwd
     */
    public void fwdByTime(double time, double str){
        if (str > Constants.MOTOR_MAX_SPEED) str = Constants.MOTOR_MAX_SPEED;
        if (str < -Constants.MOTOR_MAX_SPEED) str = -Constants.MOTOR_MAX_SPEED;
        ElapsedTime rt = new ElapsedTime();
        while(rt.seconds() <= time && opMode.opModeIsActive()) {
            drive(str, 0.0, 0.0);
        }
        stop();
    }

    /**
     * Drive forward or backward for half a second
     * @param str negative is fwd
     */
    public void nudge(double str) {
        if (str > Constants.MOTOR_MAX_SPEED) str = Constants.MOTOR_MAX_SPEED;
        if (str < -Constants.MOTOR_MAX_SPEED) str = -Constants.MOTOR_MAX_SPEED;
        ElapsedTime rt = new ElapsedTime();
        while(rt.seconds() < 0.5 && opMode.opModeIsActive())
            drive(str, 0.0, 0.0);
        stop();
    }

    /**
     * Reverses the robot until it reaches the requested distance
     * @param distance requested distance in inches
     */
    public void backUpToWall(double distance){
        while(rearDistance.getDistance() >= distance && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance());
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
            drive(0.3, 0.0, 0.0);
        }
        stop();
    }

    // ----- TURN COMMANDS -----

    public void turnRobotByDegree(double target) {
        double targetAngle = imu.getZAngle() + target;
        if(targetAngle > 180) {
            targetAngle -= 360;
        }

        while(Math.abs(targetAngle - imu.getZAngle()) >= 1 && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //drive(0.0, 0.0, Math.toRadians(getIMU().getZAngle()));
            drive(0,0,-.3);

        }
        stop();
    }

    /**
     * Rotates robot to its imu's 0 deg heading
     */
    public void turnToZero(){
        while(Math.abs(imu.getZAngle()) >= 2 && opMode.opModeIsActive()) {
            drive(0.0, 0.0, Math.toRadians(imu.getZAngle()));
        }
        stop();
    }

    /**
     * Uses distance sensors to scan area for prop and faces the claw in that direction
     */
    public void faceTheProp(){
        //TODO Use the left and right sensors to determine if it's left, right, or failing those, front
        while(rearDistance.getDistance() >= 8 && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance());
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
            drive(0.0, 0.0, 0.3);
        }
        stop();

        turnRobotByDegree(180);
    }

    // ----- STRAFE -----
    /**
     * Strafes until it sees a wall within 5 inches or until 4.75 seconds pass. Positive to the right
     * and negative to the left.
     * @param str The distance in inches to the wall
     */
    public void strafeUntilWall(double str){
        if (str > Constants.MOTOR_MAX_SPEED) str = Constants.MOTOR_MAX_SPEED;
        if (str < -Constants.MOTOR_MAX_SPEED) str = -Constants.MOTOR_MAX_SPEED;

        // are we going left or right. Use the right sensor
        DistanceSensor ds = str > 0 ? rightDistance : leftDistance;
        ElapsedTime rt = new ElapsedTime();
        while(ds.getDistance(DistanceUnit.INCH) >= 5 && rt.seconds() <= 4.75 && opMode.opModeIsActive()) {
            drive(0, str,0);
        }
        stop();
    }


    // ----- APRIL TAG -----

    /**
     * Positions robot ready to drop pixel
     * @param alignment target to align to based on randomized field config
     * @return
     */
    public boolean alignToAprilTag(AprilTagToAlign alignment) {
        if(telemetry != null) {
            switch (alignment) {
                case LEFT: telemetry.addData("Aligning To", "Left"); break;
                case CENTER: telemetry.addData("Aligning To", "Center"); break;
                case RIGHT: telemetry.addData("Aligning To", "Right"); break;
            }
        }

        List<AprilTagDetection> detections = camera.getDetections();

        // Ensure that there is at least one detection
        if(detections.size() == 0) {
            if(telemetry != null)
                telemetry.addData("Detections", "Did not find a tag");
            // Use the last stored values
            drive(0.0d,
                    Range.clip((-lastAprilTagStrafe) / Constants.APRIL_TAG_PRECISION,
                            -Constants.APRIL_TAG_MAX_SPEED, Constants.APRIL_TAG_MAX_SPEED),
                    0.0d
            );
            return lastAprilTagStrafe <= Constants.INPUT_THRESHOLD;
        }
        AprilTagDetection activeDetection = null;

        // check turning before everything else, since the detections.size() == 0 check will ensure it stays in line
        // Any detection will work for this part
        if(detections.get(0).ftcPose.yaw <= Constants.ANGLE_THRESHOLD) {
            drive(0.0d, 0.0d,
                    Range.clip(-Math.toRadians(detections.get(0).ftcPose.yaw),
                            -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED)
            );
            // Every time we have an april tag we should set this to ensure we remember where we are
            lastAprilTagStrafe = activeDetection.ftcPose.x;
            lastAprilTagYaw = activeDetection.ftcPose.yaw;
            return true;
        }
        // We have now ensured we are at the correct angle

        // Find case for if we have the correct detection
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

        // if previous case does not find correct tag
        if(activeDetection == null) {
            if(telemetry != null)
                telemetry.addData("Detections", "Found wrong tag");
            activeDetection = detections.get(0);
            switch (alignment) {
                case LEFT:
                    drive(0, Constants.APRIL_TAG_MAX_SPEED, 0);
                    break;

                case RIGHT:
                    drive(0, -Constants.APRIL_TAG_MAX_SPEED, 0 );
                    break;

                case CENTER:
                    if(activeDetection.metadata.name.toLowerCase().contains("left"))
                        drive(0,-Constants.APRIL_TAG_MAX_SPEED, 0);

                    else
                        drive(0, Constants.APRIL_TAG_MAX_SPEED, 0);
                    break;
            }
            // Every time we have an april tag we should set this to ensure we remember where we are
            lastAprilTagStrafe = activeDetection.ftcPose.x;
            lastAprilTagYaw = activeDetection.ftcPose.yaw;

            return true;
        }

        // We have now ensured that activeDetection represents the tag we are looking for
        if(telemetry != null)
            telemetry.addData("Detections", "Found correct tag");
        stop();
        // Every time we have an april tag we should set this to ensure we remember where we are
        lastAprilTagStrafe = activeDetection.ftcPose.x;
        lastAprilTagYaw = activeDetection.ftcPose.yaw;
        return true;
    }

    public void backUpUntilAprilTag(){
        while(camera.getDetections().size() == 0 && opMode.opModeIsActive()) {
            drive(-0.2, 0.0, 0.0);
        }
        stop();
    }

    /**
     * For reference
     * @param alignment the AprilTagToAlign
     * @return true if this method can be called again without adjustment
     */
    public boolean oldAlignToAprilTag(AprilTagToAlign alignment) {
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
                    drive(0, Constants.APRIL_TAG_MAX_SPEED, 0);
                    break;

                case RIGHT:
                    drive(0, -Constants.APRIL_TAG_MAX_SPEED, 0 );
                    break;

                case CENTER:
                    if(activeDetection.metadata.name.toLowerCase().contains("left"))
                        drive(0,-Constants.APRIL_TAG_MAX_SPEED, 0);

                    else
                        drive(0, Constants.APRIL_TAG_MAX_SPEED, 0);
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
        double turn = Range.clip((-activeDetection.ftcPose.yaw / Constants.APRIL_TAG_PRECISION),
                -Constants.APRIL_TAG_MAX_SPEED, Constants.APRIL_TAG_MAX_SPEED);
        telemetry.addData("Position", "Finalizing");
        // TODO: If we keep headbutting the board, insert a distance sensor check here and override the below forward calculation
        double forward = 0; //Range.clip((activeDetection.ftcPose.y - APRIL_TAG_DISTANCE_TARGET) / APRIL_TAG_PRECISION, -APRIL_TAG_MAX_SPEED, APRIL_TAG_MAX_SPEED);
        // Reversed since the camera is on the back of the robot
        double strafe = Range.clip((-activeDetection.ftcPose.x) / Constants.APRIL_TAG_PRECISION,
                -Constants.APRIL_TAG_MAX_SPEED, Constants.APRIL_TAG_MAX_SPEED);

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