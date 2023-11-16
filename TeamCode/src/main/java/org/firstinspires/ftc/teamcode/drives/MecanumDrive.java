package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive extends Drivetrain {
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    private double fieldCentricTarget = 0.0d;

    private boolean isGyroLocked = false;
    private boolean isTargetSet = false;
    private double gyroTarget = 0.0d;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            telemetry.addData("Mode", "Field Centric");
        } else
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
//;
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

    @Override
    public void turnRobotToAngle(double target) {
        // NOTE: Negative return values will increase the gyro's value
        boolean wasFieldCentric = isFieldCentric;
        isFieldCentric = false;

        int ENOUGH_CHECKS = 15; // how many times do we pass our target until we're satisfied?
        int check = 0;
        // determine the error
        double error = 0; // target - drive.gyro.getAngle();
        //TODO note time and cut off at like 3 seconds
        while(ENOUGH_CHECKS > check) {
            // determine the power output neutral of direction
            double output = Math.abs(error / target) * MOTOR_MAX_SPEED;
            if (output > MOTOR_MAX_SPEED) output = MOTOR_MAX_SPEED;

            // determine the direction
            // if I was trying to go a positive angle change from the start
            if (target > 0) {
                if (error > 0)
                    drive(0, 0, -output);
                else
                    drive(0, 0, output);
            }
            // if I was trying to go a negative angle from the start
            else{
                if (error < 0)
                    drive(0, 0, output);
                else
                    drive(0, 0, -output);
            }
            // ARE WE THERE YET?
            if (Math.abs(error) < 2) check++;

        } // end while loop

        // shut down motors
        stop();
        // restore previous fieldCentric state
        isFieldCentric = wasFieldCentric;

    }
}