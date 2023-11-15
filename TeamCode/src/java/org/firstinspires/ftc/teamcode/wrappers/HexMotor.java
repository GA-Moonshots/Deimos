package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;

public interface HexMotor extends DcMotor, RevRoboticsCoreHexMotor {
    // Inteface to merge functionality bewteen both
}
