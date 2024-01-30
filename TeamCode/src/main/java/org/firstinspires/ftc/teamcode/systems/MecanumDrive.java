/*
 * MecanumDrive Class - FTC Robot Mecanum Drive Subsystem
 *
 * This class represents the Mecanum drive subsystem on an FTC robot.
 * It includes methods for controlling the Mecanum drive using encoders, gyroscope, and distance sensors.
 *
 * Author: Micheal + [everyone who worked on it]
 * Last Modified: 12/8/2023 10:59am
 * Version: [Self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - LinearOpMode (import from com.qualcomm.robotcore.eventloop.opmode)
 *     - MecanumDrive
 *
 * Enums:
 *   - AprilTagToAlign: LEFT, CENTER, RIGHT, NONE
 *   - HowToMove: LEFT, RIGHT, BACK, ROTATE_LEFT, ROTATE_RIGHT
 *
 * Subsystem Assets:
 *   - DcMotor leftFront
 *   - DcMotor rightFront
 *   - DcMotor leftBack
 *   - DcMotor rightBack
 *   - DistanceSensor rearDistance
 *   - DistanceSensor leftDistance
 *   - DistanceSensor rightDistance
 *   - Camera camera
 *   - IMU imu
 *   - Telemetry telemetry
 *   - double gyroTarget
 *   - double runawayRobotShield
 *   - double lastAprilTagYaw
 *   - double lastAprilTagStrafe
 *   - double fieldCentricTarget
 *   - boolean isFieldCentric
 *   - boolean isGyroLocked
 *   - boolean isTargetSet
 *
 * Methods:
 *   Constructors:
 *     - MecanumDrive(LinearOpMode opMode): Initializes the Mecanum drive subsystem with sensors and motors.
 *
 *   State Commands:
 *     - makeRobotCentric(): Switches the drive to robot-centric mode.
 *     - makeFieldCentric(): Switches the drive to field-centric mode.
 *     - toggleFieldCentric(): Toggles between field-centric and robot-centric modes.
 *     - resetFieldCentricTarget(): Resets the field-centric target based on the gyroscope reading.
 *     - postDistanceReadouts(): Displays distance sensor readings in telemetry.
 *
 *   Core Drive Commands:
 *     - drive(double forward, double strafe, double turn): Drives the robot with mecanum wheels using specified inputs.
 *     - stop(): Stops the drive from moving.
 *
 *   Autonomous Drive Commands:
 *     - autonomouslyDriveByTime(double forward, double strafe, double turn, double time): Drives autonomously for a specified time.
 *     - autonomouslyMove(double strength, double target, HowToMove side, double maxTime): Moves autonomously based on sensor feedback.
 *     - maintainStrafe(DistanceSensor sensor, double strength, double target): Feedback loop for maintaining lateral position.
 *     - maintainForward(DistanceSensor sensor, double strength, double target): Feedback loop for maintaining forward position.
 *     - maintainTurn(double strength, double target): Feedback loop for maintaining robot's orientation.
 *
 *   Friendly Autonomous Pass-Through Commands:
 *     - gotoBackDistance(double str, double target, double maxTime): Moves backward autonomously to a specified distance.
 *     - gotoRightDistance(double str, double target, double maxTime): Moves right autonomously to a specified distance.
 *     - gotoLeftDistance(double str, double target, double maxTime): Moves left autonomously to a specified distance.
 *
 *   This Year's Game Commands:
 *     - faceTheProp(double str): Turns the robot to face a prop based on sensor readings.
 *     - circleScanForProp(double str, HowToMove movement, double maxTime): Scans in a circular motion to locate a prop.
 *
 */

package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.IMU;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.sensors.Camera;

public class MecanumDrive {
    // USEFUL ENUMS


    public enum HowToMove {
        LEFT,
        RIGHT,
        BACK,
        ROTATE_LEFT,
        ROTATE_RIGHT
    }

    // MOTORS
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;

    // SENSORS
    public DistanceSensor rearDistance;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;
    public Camera camera;
    public IMU imu;
    public Telemetry telemetry;

    // INSTANCE VARIABLES
    private final LinearOpMode opMode;
    private double gyroTarget = 0.0d;
    private double runawayRobotShield = -1;
    private double lastAprilTagYaw = 0.0d;
    private double lastAprilTagStrafe = 0.0d;
    private double fieldCentricTarget = 0.0d;
    private boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;


    public MecanumDrive(@NonNull LinearOpMode opMode) {
        this.imu = new IMU(opMode.hardwareMap);
        this.telemetry = opMode.telemetry;
        this.camera = new Camera(opMode.hardwareMap, opMode.telemetry);
        this.opMode = opMode;

        leftFront = opMode.hardwareMap.get(DcMotorEx.class, Constants.LEFT_FRONT_NAME);
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_FRONT_NAME);
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, Constants.LEFT_BACK_NAME);
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_BACK_NAME);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearDistance = new DistanceSensor(opMode, Constants.REAR_DIST_NAME);
        rightDistance = new DistanceSensor(opMode, Constants.RIGHT_DIST_NAME);
        leftDistance = new DistanceSensor(opMode, Constants.LEFT_DIST_NAME);

        fieldCentricTarget = imu.getZAngle();
    }
    // ------ STATE COMMANDS -------
    public void makeRobotCentric() {
        isFieldCentric = false;
    }

    public void makeFieldCentric() {
        isFieldCentric = true;
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void resetFieldCentricTarget() {
        fieldCentricTarget = imu.getZAngle();
    }

    public void postDistanceReadouts() {
        telemetry.addData("Left Dist", leftDistance.getDistance());
        telemetry.addData("Right Dist", rightDistance.getDistance());
        telemetry.addData("Rear Dist", rearDistance.getDistance());
    }

    // ---- CORE DRIVE COMMANDS ----
    // ---- CORE DRIVE COMMANDS ----
    // ---- CORE DRIVE COMMANDS ----

    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    /**
     * Translates desired motion into mecanum commands
     * @param forward negative is forward
     * @param strafe lateral movement
     * @param turn positive is clockwise
     */
    public void drive(double forward, double strafe, double turn) {

        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more: https://www.geogebra.org/m/fmegkksm
            double diff = fieldCentricTarget - imu.getZAngle();
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(diff)) - strafe * Math.sin(Math.toRadians(diff));
            strafe = temp * Math.sin(Math.toRadians(diff)) + strafe * Math.cos(Math.toRadians(diff));
            if(telemetry != null)
                telemetry.addData("Mode", "Field Centric");
        } else if(telemetry != null)
            telemetry.addData("Mode", "Robot Centric");

        // lock onto a gyro heading if not turning but is driving
        isGyroLocked = turn <= Constants.INPUT_THRESHOLD
                && Math.hypot(forward, strafe) >= Constants.INPUT_THRESHOLD;
        double boost = 0;
        if(isGyroLocked && !isTargetSet) {
            gyroTarget = imu.getYAngle();
            isTargetSet = true;
        } else if(!isGyroLocked) {
            isTargetSet = false;
        }

        //if(isTargetSet && isGyroLocked)
            //boost = -(gyroTarget - imu.getYAngle()) / Constants.GYRO_BOOST_FACTOR;

        // forward is reversed (flight stick) and boost values should match the turn
        // the mecanum drive is a X instead of a diamond
        double leftFrontPower = -forward + strafe + turn + boost;
        double rightFrontPower = forward + strafe + turn + boost;
        double leftBackPower = -forward - strafe + turn + boost;
        double rightBackPower = forward - strafe + turn + boost;

        // if an input exceeds max, everything is proportionally reduced to keep balanced
        double powerScale = Math.max(1,
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

        // ?? if we clip the ranges below, why are we multiplying them here?
        leftFrontPower *= Constants.MOTOR_MAX_SPEED;
        leftBackPower *= Constants.MOTOR_MAX_SPEED;
        rightBackPower *= Constants.MOTOR_MAX_SPEED;
        rightFrontPower *= Constants.MOTOR_MAX_SPEED;

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

    /**
     * Clips and executes given motor speeds
     */
    protected void drive(double m1, double m2, double m3, double m4) {
        leftFront.setPower(Range.clip(m1, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
    }

    // ---- AUTONOMOUS DRIVE COMMANDS ----
    // ---- AUTONOMOUS DRIVE COMMANDS ----
    // ---- AUTONOMOUS DRIVE COMMANDS ----

    /**
     * Rotates robot to its imu's 0 degree heading adjusted by fieldCentricTarget
     */
    public void goToZeroAngle() {
        while (Math.abs(imu.getZAngle() - fieldCentricTarget) >= 2 && opMode.opModeIsActive()) {
            drive(0.0, 0.0, 0.7 * Math.toRadians(imu.getZAngle() - fieldCentricTarget));
        }
        stop();
    }

    public void autonomouslyDriveByTime(double forward, double strafe, double turn, double time) {
        drive(forward, strafe, turn);
        opMode.sleep((long) time * 1000);
        stop();
    }

    /**
     * @param strength The speed to move at. This should always be positive
     * @param target The distance (or angle) that the sensor attempts to go to
     * @param side The sensor (or rotation direction) to listen to and move in
     * @param maxTime The maximum time in seconds this function is allowed to run for
     */
    public void autonomouslyMove(double strength, double target, HowToMove side, double maxTime) {
        ElapsedTime runtime = new ElapsedTime();
        while(opMode.opModeIsActive() && runtime.seconds() <= maxTime) {
            switch (side) {
                case LEFT:
                    if(maintainStrafe(leftDistance, strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case RIGHT:
                    if(maintainStrafe(rightDistance, -strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case BACK:
                    if(maintainForward(rearDistance, strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case ROTATE_LEFT:
                    if(maintainTurn(-strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case ROTATE_RIGHT:
                    if(maintainTurn(strength, target)) {
                        stop();
                        return;
                    }
                    break;
            }
        }
        stop();
    }

    /**
     * A repeatedly called feedback control loop that uses a proportional gain
     * to control the robot's strafing motion and reach a specific lateral position.
     * @param sensor the distance sensor used to determine completion
     * @param strength motor power
     * @param target desired distance
     * @return true if complete, false if needs to continue
     */
    protected boolean maintainStrafe(DistanceSensor sensor, double strength, double target) {
        // note the current distance
        double distance = sensor.doubleCheckDistance();

        // one-time, initial determination if we're approaching or leaving the wall
        if(runawayRobotShield == -1) {
            // if we're leaving, cap the allowed distance
            runawayRobotShield = distance > target ? 0 : target + Constants.DISTANCE_THRESHOLD;
        }
        // IS IT TIME TO STOP?
        if(Math.abs(distance - target) <= Constants.DISTANCE_THRESHOLD ||
                (runawayRobotShield != 0 && distance > runawayRobotShield)) {
            stop();
            runawayRobotShield = -1; // reset the shield
            return true;
        }

        // CONTINUE DRIVING
        // If we are further away than we should be (distance > target) on left side,
        // we should go back to the left (ie negative value)
        drive(0.0d, Constants.KP * strength * (target - sensor.getDistance()), 0.0d);
        return false;
    }

    /**
     * A repeatedly called feedback control loop using proportional gain
     * @param sensor distance sensor used to determine progress
     * @param strength motor speed, negative is forward
     * @param target desired distance
     * @return true if completed, false if motion needs to continue
     */
    protected boolean maintainForward(DistanceSensor sensor, double strength, double target) {
        // note the current distance
        double distance = sensor.doubleCheckDistance();

        // one-time, initial determination if we're approaching or leaving the wall
        if(runawayRobotShield == -1) {
            // if we're leaving, cap the allowed distance
            runawayRobotShield = distance > target ? 0 : target + Constants.DISTANCE_THRESHOLD;
        }
        // IS IT TIME TO STOP?
        if(Math.abs(distance - target) <= Constants.DISTANCE_THRESHOLD ||
                (runawayRobotShield != 0 && distance > runawayRobotShield)) {
            stop();
            runawayRobotShield = -1; // reset the shield
            return true;
        }

        // CONTINUE DRIVING
        // If we are further away than we should be (distance > target),
        // we should go backwards (positive value)
        drive(Constants.KP * strength * (sensor.getDistance() - target), 0.0d, 0.0d);
        return false;
    }

    /**
     * A repeatedly called feedback control loop for rotating the robot
     * @param strength motor speed
     * @param target desired gyroscope reading
     * @return true if motion is completed, false if needs to continue
     */
    protected boolean maintainTurn(double strength, double target) {
        // Wrap target
        target %= 180;

        if(Math.abs(imu.getZAngle() - target) <= Constants.ANGLE_THRESHOLD) {
            stop();
            return true;
        }

        drive(0.0d, 0.0d, strength);
        return false;
    }

    // ---- FRIENDLY AUTONOMOUS PASS-THROUGH COMMANDS ----
    // ---- FRIENDLY AUTONOMOUS PASS-THROUGH COMMANDS ----
    // ---- FRIENDLY AUTONOMOUS PASS-THROUGH COMMANDS ----

    public void gotoBackDistance(double str, double target, double maxTime) {
        autonomouslyMove(str, target, HowToMove.BACK, maxTime);
    }

    public void gotoBackDistance(double target, double maxTime) {
        gotoBackDistance(0.3, target, maxTime);
    }

    public void gotoBackDistance(double target) {
        gotoBackDistance( target, 3);
    }

    public void gotoRightDistance(double str, double target, double maxTime) {
        autonomouslyMove(str, target, HowToMove.RIGHT, maxTime);
    }

    public void gotoRightDistance(double target, double maxTime) {
        gotoRightDistance(0.3, target, maxTime);
    }

    public void gotoRightDistance(double target) {
        gotoRightDistance( target, 3);
    }

    public void gotoLeftDistance(double str, double target, double maxTime) {
        autonomouslyMove(str, target, HowToMove.LEFT, maxTime);
    }

    public void gotoLeftDistance(double target, double maxTime) {
        gotoLeftDistance(0.3, target, maxTime);
    }

    public void gotoLeftDistance(double target) {
        gotoLeftDistance( target, 3);
    }

    public void gotoAngle(double target, HowToMove move, double maxTime) {
        autonomouslyMove(0.3, target, move, maxTime);
    }

    public void gotoAngle(double target, HowToMove move) {
        gotoAngle(target, move, 2);
    }

    public void gotoAngle(double target, double maxTime) {
        if(imu.getZAngle() - target > 0)
            gotoAngle(target, HowToMove.ROTATE_LEFT, maxTime);
        else
            gotoAngle(target, HowToMove.ROTATE_RIGHT, maxTime);
    }

    public void gotoAngle(double target) {
        gotoAngle(target, 2);
    }

    // ---- THIS YEAR'S GAME ----
    // ---- THIS YEAR'S GAME ----
    // ---- THIS YEAR'S GAME ----

    public Camera.AprilTagToAlign faceTheProp(double str) {
        return faceTheProp(str, 1.5);
    }

    public Camera.AprilTagToAlign faceTheProp(double str, double maxTime) {
        boolean isLeft = false;
        boolean isRight = false;

        ElapsedTime rt = new ElapsedTime();
        while(!(isLeft || isRight) && rt.seconds() <= maxTime && opMode.opModeIsActive()) {
            // check left
            if(leftDistance.doubleCheckDistance() <= 10) isLeft = true;
                //check right
            else if(rightDistance.doubleCheckDistance() <= 10) isRight = true;
            else drive(-0.1, 0.0, 0.0);
        }
        stop();
        // back up
        autonomouslyDriveByTime(0.1, 0.0, 0.0, 0.3);

        // turn to prop based on prev
        if(isLeft) {
            autonomouslyMove(str, 90, HowToMove.ROTATE_LEFT, 5);
            autonomouslyDriveByTime(0.0, -0.1, 0.0, 0.2);
            return Camera.AprilTagToAlign.LEFT;
        } else if(isRight) {
            autonomouslyMove(str, -90, HowToMove.ROTATE_RIGHT, 5);
            autonomouslyDriveByTime(0.0, 0.1, 0.0, 0.2);
            return Camera.AprilTagToAlign.RIGHT;
        } else {
            autonomouslyDriveByTime(0.15, 0.0, 0.0, 1);
            return Camera.AprilTagToAlign.CENTER;
        }
    }

    public void circleScanForProp(double str, HowToMove movement, double maxTime) {
        // Set the power value to the correct mode
        double realStr = movement == HowToMove.ROTATE_LEFT ? -str : str;
        // Add timing? May not be required for this function
        ElapsedTime rt = new ElapsedTime();
        while(rearDistance.getDistance() >= 8 && opMode.opModeIsActive() && rt.seconds() <= maxTime) {
            telemetry.addData("Rear Distance", rearDistance.getDistance());
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
            drive(0.0, 0.0, realStr);
        }
        stop();

        telemetry.addData("Ed sez", "I saw the prop :) good job wahoo");
        telemetry.update();

        // Find the new target angle to go to
        double targetAngle = (imu.getZAngle() - fieldCentricTarget + 180);
        if(targetAngle > 180) {
            targetAngle -= 360;
        }
        if(targetAngle < -180) {
            targetAngle += 360;
        }

        // Rotate the opposite way
        HowToMove newMovement = (movement == HowToMove.ROTATE_LEFT) ? HowToMove.ROTATE_LEFT : HowToMove.ROTATE_RIGHT;

        autonomouslyMove(str, targetAngle, newMovement, maxTime);
    }
}