package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    public Rev2mDistanceSensor distanceSensor;
    private LinearOpMode opMode;

    public DistanceSensor(LinearOpMode opMode, String name) {
        com.qualcomm.robotcore.hardware.DistanceSensor converter;
        this.opMode = opMode;
        converter = opMode.hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, name);
        // casting like this is the official recommendation:
        // https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SensorREV2mDistance.java
        distanceSensor = (Rev2mDistanceSensor)converter;
    }

    /**
     * @return distance in inches
     */
    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    /**
     * Retrieves a more reliable distance measurement by taking three readings if the first two
     * have a difference of greater than an inch. The outlying reading is discarded
     * @return The average distance after removing any divergent readings.
     */
    public double doubleCheckDistance() {
        double firstCheck = getDistance();
        opMode.sleep(25);
        double secondCheck = getDistance();
        // Calculate absolute difference between first and second readings
        double delta = Math.abs(firstCheck - secondCheck);

        if (delta > 1) {
            opMode.sleep(25);
            // Get the third distance reading
            double thirdCheck = getDistance();

            // Determine the outlier based on proximity to other readings
            double outlier = Math.abs(firstCheck - secondCheck) > Math.abs(secondCheck - thirdCheck) ?
                    firstCheck : thirdCheck;

            // Exclude the outlier and calculate the average of remaining values
            return (firstCheck + secondCheck + thirdCheck - outlier) / 2;
        } else {
            return secondCheck;
        }
    }

    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
}
