package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    public Rev2mDistanceSensor distanceSensor;

    public DistanceSensor(HardwareMap hardwareMap, String name) {
        com.qualcomm.robotcore.hardware.DistanceSensor converter;
        converter = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, name);
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

    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
}
