/*
 * IMU Class - FTC Robot Inertial Measurement Unit
 *
 * This class represents the Inertial Measurement Unit (IMU) subsystem on an FTC robot. It uses the BNO055IMU hardware
 * to measure orientation and velocity in three dimensions (X, Y, Z). The class provides methods to retrieve angles,
 * angular velocities, and positions from the IMU.
 *
 * Author: [everyone who worked on it]
 * Last Modified: 12/8/2023 12:01pm
 * Version: [self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - IMU
 *       - BNO055IMU
 *
 * Subsystem Assets:
 *   - com.qualcomm.robotcore.hardware.IMU imu
 *
 * Methods:
 *   Constructors:
 *     - IMU(HardwareMap hardwareMap): Initializes the IMU subsystem with the BNO055IMU hardware. It takes the
 *       HardwareMap as a parameter.
 *
 *   Orientation Commands:
 *     - double getXAngle(): Retrieves the X angle of the internal IMU in the control panel.
 *     - double getYAngle(): Retrieves the Y angle of the internal IMU in the control panel.
 *     - double getZAngle(): Retrieves the Z angle of the internal IMU in the control panel.
 *     - double[] getAngle(): Retrieves a double array, ordered XYZ, of the angle.
 *
 *   Velocity Commands:
 *     - double getXVelocity(): Retrieves the X axis velocity of the control panel.
 *     - double getYVelocity(): Retrieves the Y axis velocity of the control panel.
 *     - double getZVelocity(): Retrieves the Z axis velocity of the control panel.
 *     - double[] getVelocity(): Retrieves an ordered XYZ array of the control panel's current velocity.
 *
 *   Position Commands:
 *     - double getXPosition(): Retrieves the X axis position of the control panel.
 *     - double getYPosition(): Retrieves the Y axis position of the control panel.
 *     - double getZPosition(): Retrieves the Z axis position of the control panel.
 *     - double[] getPosition(): Retrieves an ordered XYZ array of the control panel's current position.
 *
 */


package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constants;

public class IMU {
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    public com.qualcomm.robotcore.hardware.IMU imu;

    public IMU(HardwareMap hardwareMap) {
        com.qualcomm.robotcore.hardware.IMU.Parameters parameters =
                new com.qualcomm.robotcore.hardware.IMU.Parameters(
                    new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ));

        imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, Constants.IMU_NAME);
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
