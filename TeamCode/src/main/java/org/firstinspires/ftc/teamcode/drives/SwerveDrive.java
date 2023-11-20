package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveDrive extends Drivetrain {
    public final DcMotor llMotor;   // +Power = left wheel turning left
    public final DcMotor lrMotor;  // +Power = left wheel turning right
    public final DcMotor rlMotor;  // +Power = right wheel turning right
    public final DcMotor rrMotor; // + Power = right wheel turning left
    private boolean fieldCentric = true;

    /**
     * Returns the wheel angle
     * @param left The left motor for the wheel
     * @param right the right motor for the wheel
     * @return The angle of the wheel made up of the left and right motors
     */
    private double getWheelAngle(DcMotor left, DcMotor right) {
        double angle = 360 * ((left.getCurrentPosition() - right.getCurrentPosition()) / SWERVE_ENCODER_COUNTS_PER_REV ) % SWERVE_ENCODER_COUNTS_PER_REV;
        if(getFieldCentric()) return angle + getIMU().getZAngle();
        return angle;
    }

    //Field Centric related Methods
    public void makeRobotCentric() {
        fieldCentric = false;
    }
    public void switchMode() {
        fieldCentric = !fieldCentric;
    }
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public SwerveDrive(LinearOpMode opMode) {
        super(opMode, new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        llMotor = opMode.hardwareMap.get(DcMotor.class, "llMotor");
        lrMotor = opMode.hardwareMap.get(DcMotor.class, "lrMotor");
        rlMotor = opMode.hardwareMap.get(DcMotor.class, "rlMotor");
        rrMotor = opMode.hardwareMap.get(DcMotor.class, "rrMotor");
    }

    @Override
    public void drive(double forward, double strafe, double turn) {
        // Useful Variables for later
        double rightWheelAngle = getWheelAngle(rlMotor, rrMotor);
        double leftWheelAngle = getWheelAngle(llMotor, lrMotor);
        double leftRotPower, rightRotPower;
        double multiplier = 1;

        // ______________
        // -- ROTATION --
        if(Math.abs(turn) >= Constants.INPUT_THRESHOLD) {
            // If Wheel's forward face is away from the 0 mark, reverse rotation
            if(Math.abs(leftWheelAngle) >= 90) {
                multiplier = -1;
            }

            // multiplier flips rotation direction; Rest calculates how much rotation we need
            // https://www.geogebra.org/calculator/ucaxvmtw
            leftRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-leftWheelAngle));
            rightRotPower = SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(-rightWheelAngle));

            drive(
                    multiplier * (-turn + leftRotPower),
                    multiplier * (-turn - leftRotPower),
                    multiplier * (rightRotPower - turn),
                    multiplier * (-turn - rightRotPower));
            return;
        }

        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(imu.getZAngle())) - strafe * Math.sin(Math.toRadians(imu.getZAngle()));
            strafe = temp * Math.sin(Math.toRadians(imu.getZAngle())) + strafe * Math.cos(Math.toRadians(imu.getZAngle()));
        }

        // Calculate magnitude of joystick being pushed
        double radius = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));

        if(radius <= Constants.INPUT_THRESHOLD) {
            return;
        }

        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Left is reversed due to gearing
        double llPower = -radius;
        double lrPower = -radius;
        double rlPower = radius;
        double rrPower = radius;

        // Point wheels in correct direction
        double targetAngle = Math.toDegrees(Math.atan2(strafe, forward));
        double leftAngleDiff = targetAngle - leftWheelAngle;
        double rightAngleDiff = targetAngle - rightWheelAngle;

        if(telemetry != null) {
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Left Wheel Diff", leftAngleDiff);
            telemetry.addData("Right Wheel Diff", rightAngleDiff);
        }

        // Wheel Dead Zones
        if(Math.abs(leftAngleDiff) <= ANGLE_TOLERANCE) {
            leftAngleDiff = 0.0d;
        }

        if(Math.abs(rightAngleDiff) <= ANGLE_TOLERANCE) {
            rightAngleDiff = 0.0d;
        }

        // Reverse power if closer to go to 180 than 0
        if (Math.abs(leftAngleDiff) >= 90){
            multiplier = -1;
        }

        llPower *= multiplier;
        lrPower *= multiplier;
        rlPower *= multiplier;
        rrPower *= multiplier;

        leftRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(leftAngleDiff));
        llPower += leftRotPower;
        lrPower -= leftRotPower;

        rightRotPower = multiplier * SWERVE_WHEEL_ROT_MULTIPLIER * Math.sin(Math.toRadians(rightAngleDiff));
        rlPower += rightRotPower;
        rrPower -= rightRotPower;

        // Add Forward Drive power
        drive(llPower, lrPower, rlPower, rrPower);
    }

    @Override
    public void drive(double m1, double m2, double m3, double m4) {
        llMotor.setPower(Range.clip(m1, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        lrMotor.setPower(Range.clip(m2, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rlMotor.setPower(Range.clip(m3, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
        rrMotor.setPower(Range.clip(m4, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED));
    }




    public void setDistanceToTravel(double distance) {
        llMotor.setTargetPosition(
                llMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );

        lrMotor.setTargetPosition(
                lrMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );

        rlMotor.setTargetPosition(
                rlMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );

        rrMotor.setTargetPosition(
                rrMotor.getCurrentPosition() - (int)(distance * SWERVE_ENCODER_COUNTS_PER_INCH)
        );
        
        llMotor.setPower(MOTOR_MAX_SPEED);
        lrMotor.setPower(MOTOR_MAX_SPEED);
        rlMotor.setPower(MOTOR_MAX_SPEED);
        rrMotor.setPower(MOTOR_MAX_SPEED);
        llMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
