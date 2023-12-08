/*
 * Elevator Class - FTC Robot Elevator Subsystem
 *
 * This class represents the elevator subsystem on an FTC robot.
 * It includes methods for controlling the elevator's movement and locking mechanism.
 *
 * Author: [everyone who worked on it]
 * Last Modified: 12/8/2023 10:07am
 * Version: [self explanatory, but idk what version it is]
 *
 * Class Hierarchy:
 *   - LinearOpMode (import from com.qualcomm.robotcore.eventloop.opmode)
 *     - Elevator
 *
 * Subsystem Assets:
 *   - DcMotor motor
 *   - LinearOpMode opMode
 *   - Servo leftServo
 *   - Servo rightServo
 *
 * State Variables:
 *   - boolean isLocked
 *
 * Constants:
 *   - Constants.ELEVATOR_MOTOR_NAME
 *   - Constants.ELEVATOR_LEFT_SERVO_NAME
 *   - Constants.ELEVATOR_RIGHT_SERVO_NAME
 *   - Constants.LOCK_OFF_POSITION
 *   - Constants.LOCK_ON_POSITION
 *   - Constants.ELEVATOR_MAX_SPEED
 *
 * Methods:
 *   - Elevator(LinearOpMode opMode): Constructor to initialize the elevator subsystem.
 *   - void move(double str): Moves the elevator based on the given power.
 *   - void toggleLock(): Toggles the lock mechanism on/off.
 *   - void stop(): Stops the elevator motor.
 */

package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Elevator {
    private DcMotor motor;
    private LinearOpMode opMode;

    private Servo leftServo;
    private Servo rightServo;

    private boolean isLocked = false;


    public Elevator(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftServo = opMode.hardwareMap.get(Servo.class, Constants.ELEVATOR_LEFT_SERVO_NAME);
        rightServo = opMode.hardwareMap.get(Servo.class, Constants.ELEVATOR_RIGHT_SERVO_NAME);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(Constants.LOCK_OFF_POSITION);
        rightServo.setPosition(Constants.LOCK_OFF_POSITION);
    }

    public void move(double str) {
        motor.setPower(Range.clip(str, -Constants.ELEVATOR_MAX_SPEED, Constants.ELEVATOR_MAX_SPEED));
    }
    public void toggleLock() {
        if(isLocked) {
            leftServo.setPosition(Constants.LOCK_OFF_POSITION);
            rightServo.setPosition(Constants.LOCK_OFF_POSITION);
        } else {
            leftServo.setPosition(Constants.LOCK_ON_POSITION);
            rightServo.setPosition(Constants.LOCK_ON_POSITION);
        }
        isLocked = false;
    }

    public void stop() {
        motor.setPower(0.0d);
    }
}
