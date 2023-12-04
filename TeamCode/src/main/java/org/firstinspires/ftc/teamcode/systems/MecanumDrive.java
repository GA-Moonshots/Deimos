package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.IMU;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.sensors.Camera;

public class MecanumDrive {
    // USEFUL ENUMS
    public enum AprilTagToAlign {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public enum HowToMove {
        LEFT,
        RIGHT,
        BACK,
        ROTATE_LEFT,
        ROTATE_RIGHT
    }

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
    private boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;


    public MecanumDrive(@NonNull LinearOpMode opMode) {
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

    // ---- CORE DRIVE COMMANDS ----
    /**
     * Stops the drive from moving
     */
    public void stop() {
        drive(0.0d, 0.0d, 0.0d, 0.0d);
    }

    /**
     *
     * @param forward negative is forward
     * @param strafe
     * @param turn positive is clockwise
     */
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

    public void autoDriveByTime(double forward, double strafe, double turn, double time) {
        assert forward >= -1 && forward <= 1 && strafe >= -1 && strafe <= 1 && turn >= -1 && turn <= 1 && time > 0;

        ElapsedTime rt = new ElapsedTime();
        while(opMode.opModeIsActive() && rt.seconds() <= time) {
            drive(forward, strafe, turn);
        }
        stop();
    }

    /**
     * Blocking Autonomous drive command, with a maxTime backup system
     *
     * @param strength The speed to move at. This should always be positive
     * @param target The distance (or angle) that the sensor attempts to go to
     * @param side The sensor (or rotation direction) to listen to and move in
     * @param maxTime The maximum time in seconds this function is allowed to run for
     *
     * @return If this function stopped by distance sensors (true) or by time (false)
     */
    public void autoGoToPosition(double strength, double target, HowToMove side, double maxTime) {
        assert maxTime > 0 && strength > 0 && strength <= 1;
        ElapsedTime runtime = new ElapsedTime();
        while(opMode.opModeIsActive() && runtime.seconds() <= maxTime) {
            switch (side) {
                case LEFT:
                    if(autoStrafeBySensor(leftDistance, strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case RIGHT:
                    if(autoStrafeBySensor(rightDistance, -strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case BACK:
                    if(autoForwardBySensor(rearDistance, strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case ROTATE_LEFT:
                    if(autoTurn(-strength, target)) {
                        stop();
                        return;
                    }
                    break;
                case ROTATE_RIGHT:
                    if(autoTurn(strength, target)) {
                        stop();
                        return;
                    }
                    break;
            }
        }
        stop();
    }

    protected boolean autoStrafeBySensor(DistanceSensor sensor, double strength, double target) {
        // Check if we finished the step
        if(Math.abs(sensor.getDistance() - target) <= Constants.DISTANCE_THRESHOLD) {
            stop();
            return true;
        }

        // Continue the step
        if(sensor.getDistance() > target) {
            drive(0.0d, -strength, 0.0d);
        } else {
            drive(0.0d, strength, 0.0d);
        }
        return false;
    }

    protected boolean autoForwardBySensor(DistanceSensor sensor, double strength, double target) {
        if(Math.abs(sensor.getDistance() - target) <= Constants.DISTANCE_THRESHOLD) {
            stop();
            return true;
        }

        if(sensor.getDistance() > target) {
            drive(strength, 0.0d, 0.0d);
        } else {
            drive(-strength, 0.0d, 0.0d);
        }
        return false;
    }

    protected boolean autoTurn(double strength, double target) {
        // Wrap target
        target %= 360;

        if(Math.abs(imu.getZAngle() - target) <= Constants.ANGLE_THRESHOLD) {
            stop();
            return true;
        }

        drive(0.0d, 0.0d, strength);
        return false;
    }

    public void faceTheProp(double str, HowToMove movement, double maxTime){
        assert (str > 0 && str <= 1) && (movement == HowToMove.ROTATE_LEFT || movement == HowToMove.ROTATE_RIGHT);

        // Set the power value to the correct mode
        double realStr = movement == HowToMove.ROTATE_LEFT ? -str : str;
        // Add timing? May not be required for this function
        ElapsedTime rt = new ElapsedTime();
        //TODO: Use the left and right sensors to determine if it's left, right, or failing those, front
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
        double targetAngle = (imu.getZAngle() - fieldCentricTarget + 180) % 360;
        if(targetAngle > 180) {
            targetAngle -= 360;
        }
        if(targetAngle < -180) {
            targetAngle += 360;
        }

        // Rotate the opposite way
        HowToMove newMovement = (movement == HowToMove.ROTATE_LEFT) ? HowToMove.ROTATE_LEFT : HowToMove.ROTATE_RIGHT;

        autoGoToPosition(str, targetAngle, newMovement, maxTime);
    }

    /**
     * Rotates robot to its imu's 0 deg heading
     */
    public void goToZero() {
        while (Math.abs(imu.getZAngle() - fieldCentricTarget) >= 2 && opMode.opModeIsActive()) {
            drive(0.0, 0.0, Math.toRadians(imu.getZAngle() - fieldCentricTarget));
        }
        stop();
    }

    public void getSensorReadout() {
        telemetry.addData("Left Dist", leftDistance.getDistance());
        telemetry.addData("Right Dist", rightDistance.getDistance());
        telemetry.addData("Rear Dist", rearDistance.getDistance());
    }

    public void gotoBackDistance(double str, double target, double maxTime) {
        autoGoToPosition(str, target, HowToMove.BACK, maxTime);
    }

    public void gotoBackDistance(double target, double maxTime) {
        gotoBackDistance(0.3, target, maxTime);
    }

    public void gotoBackDistance(double target) {
        gotoBackDistance( target, 3);
    }
}