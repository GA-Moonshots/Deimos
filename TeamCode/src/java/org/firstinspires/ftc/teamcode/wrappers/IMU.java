package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class IMU {
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    public static final String NAME = "imu";

    com.qualcomm.robotcore.hardware.IMU imu;

    public IMU(HardwareMap hardwareMap, com.qualcomm.robotcore.hardware.IMU.Parameters parameters) {

        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, NAME);
        imu.initialize(parameters);
    }

    /**
     *
     * @return the X angle of the internal IMU in the control panel.
     */
    public double getXAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT).firstAngle;
    }

    /**
     *
     * @return the Y angle of the internal IMU in the control panel.
     */
    public double getYAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT).secondAngle;
    }

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, ANGLE_UNIT).thirdAngle;
    }

    /**
     *
     * @return a double array, ordered XYZ, of the angle.
     */
    public double[] getAngle() {
        double[] out = new double[3];

        out[0] = getXAngle();
        out[1] = getYAngle();
        out[2] = getZAngle();

        return out;
    }

    /**
     *
     * @return The X axis velocity of the control panel.
     */
    public double getXVelocity() {
        try {
            return ((BNO055IMU) imu).getVelocity().xVeloc;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    /**
     *
     * @return The Y axis velocity of the control panel.
     */
    public double getYVelocity() {
        try {
            return ((BNO055IMU) imu).getVelocity().yVeloc;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    /**
     *
     * @return The Z axis velocity of the control panel.
     */
    public double getZVelocity() {
        try {
            return ((BNO055IMU) imu).getVelocity().zVeloc;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    /**
     *
     * @return An ordered XYZ array of the control panel's current velocity.
     */
    public double[] getVelocity() {
        double[] out = new double[3];

        out[0] = getXVelocity();
        out[1] = getYVelocity();
        out[2] = getZVelocity();

        return out;
    }

    public double getXPosition() {
        try {
            return ((BNO055IMU) imu).getPosition().x;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    public double getYPosition() {
        try {
            return ((BNO055IMU) imu).getPosition().y;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    public double getZPosition() {
        try {
            return ((BNO055IMU) imu).getPosition().z;
        } catch(Exception e) {
            return 0.0d;
        }
    }

    public double[] getPosition() {
        double[] out = new double[3];

        out[0] = getXPosition();
        out[1] = getYPosition();
        out[2] = getZPosition();

        return out;
    }
}
