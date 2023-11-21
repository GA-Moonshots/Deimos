package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.vision.Camera;

public class MecanumDrive extends Drivetrain {
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

    // INSTANCE VARIABLES
    private double fieldCentricTarget = 0.0d;
    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;
    private double gyroTarget = 0.0d;

    /**
     * This constructor takes a reference to the aggregate object that owns it. It's a controversial
     * design decision and one that can be avoided with a command scheduler
     * @param opMode
     */
    public MecanumDrive(LinearOpMode opMode) {
        super(opMode, new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        leftFront = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearDistance = new DistanceSensor(opMode.hardwareMap, "rear");
        rightDistance = new DistanceSensor(opMode.hardwareMap, "right");
        leftDistance = new DistanceSensor(opMode.hardwareMap, "left");
        this.camera = new Camera(opMode.hardwareMap, telemetry);
        fieldCentricTarget = imu.getZAngle();
    }

    @Override
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

        double powerScale = MOTOR_MAX_SPEED * Math.max(1,
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

    @Override
    protected void drive(double m1, double m2, double m3, double m4) {
        leftFront.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }


    public void turnRobotByDegree(double target) {
        double targetAngle = getIMU().getZAngle() + target;
        if(targetAngle > 180) {
            targetAngle -= 360;
        }

        while(Math.abs(targetAngle - getIMU().getZAngle()) >= 1 && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", getIMU().getZAngle());
            telemetry.addData("Target", targetAngle);
            telemetry.update();
            //drive(0.0, 0.0, Math.toRadians(getIMU().getZAngle()));
            drive(0,0,-.3);

        }
        stop();
    }

    /**
     * Drives away from wall until rear sensor reaches the requested distance or until 3 seconds
     * have passed
     * @param distance requested distance in inches
     */
    public void fwdFromWall(double distance){
        ElapsedTime rt = new ElapsedTime();
        while(rearDistance.getDistance(DistanceUnit.INCH) <= distance && rt.seconds() <= 3
                && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", getIMU().getZAngle());
            telemetry.update();
            drive(-0.3, 0.0, 0.0);
        }
        stop();
    }

    /**
     * Reverses the robot until it reaches the requested distance
     * @param distance requested distance in inches
     */
    public void backUpToWall(double distance){
        while(rearDistance.getDistance(DistanceUnit.INCH) >= distance && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", getIMU().getZAngle());
            telemetry.update();
            drive(0.3, 0.0, 0.0);
        }
        stop();
    }

    public void turnToZero(){
        while(Math.abs(imu.getZAngle()) >= 1 && opMode.opModeIsActive()) {
            drive(0.0, 0.0, Math.toRadians(imu.getZAngle()));
        }
        stop();
    }

    /**
     * Strafes until it sees a wall within 5 inches or until 3 seconds pass. Positive to the right
     * and negative to the left.
     * @param str The distance in inches to the wall
     */
    public void strafeUntilWall(double str){
        // are we going left or right. Use the right sensor
        DistanceSensor ds = str > 0 ? rightDistance : leftDistance;
        ElapsedTime rt = new ElapsedTime();
        while(ds.getDistance(DistanceUnit.INCH) >= 5 && rt.seconds() <= 3 && opMode.opModeIsActive()) {
            drive(0, str,0);
        }
        stop();
    }

    public void faceTheProp(){
        //TODO Use the left and right sensors to determine if it's left, right, or failing those, front
        while(rearDistance.getDistance(DistanceUnit.INCH) >= 8 && opMode.opModeIsActive()) {
            telemetry.addData("Rear Distance", rearDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("IMU Angle", getIMU().getZAngle());
            telemetry.update();
            drive(0.0, 0.0, 0.3);
        }
        stop();

        turnRobotByDegree(180);
    }


    
}