/*
 * Arm Class - FTC Robot Robotic Arm Subsystem
 *
 * This class represents the robotic arm subsystem on an FTC robot.
 * It includes methods for controlling the arm's movement, wrist, claw, and roll servo.
 *
 * Author: [everyone who worked on it]
 * Last Modified: 12/8/2023 9:42am
 * Version: [self explanatory, but idk what version it is]
 *
 * Class Hierarchy:
 *   - LinearOpMode (import from com.qualcomm.robotcore.eventloop.opmode)
 *     - Arm
 *
 * Subsystem Assets:
 *   - Servo wristServo
 *   - Servo openServo
 *   - Servo rollServo
 *   - DcMotor motor
 *   - LinearOpMode opMode
 *
 * State Variables:
 *   - double wristAng
 *   - double rollPos
 *   - boolean isOpen
 *   - int offset
 *   - enum RunState { GOTO_DROPOFF, GOTO_GROUND, NONE }
 *
 * Constants:
 *   - Constants.WRIST_ON_GROUND
 *   - Constants.ROLL_MAX
 *   - Constants.ARM_MOTOR_NAME
 *   - Constants.ARM_UP_POSITION
 *   - Constants.ARM_DOWN_POSITION
 *   - Constants.ARM_MOTOR_STRENGTH
 *   - Constants.ROLL_SERVO_NAME
 *   - Constants.CLAW_CLOSED_POS
 *   - Constants.WRIST_ON_WALL
 *   - Constants.ROLL_MIN
 *   - Constants.WRIST_MIN
 *   - Constants.WRIST_MAX
 *   - Constants.WRIST_INC
 *   - Constants.CLAW_OPEN_POS
 *   - Constants.ROLL_INC
 *
 * Methods:
 *   - Arm(LinearOpMode opMode): Constructor to initialize the arm subsystem.
 *   - void move(double shoulderRot): Moves the arm based on the given shoulder rotation.
 *   - boolean goToPickUp(): Moves the arm to the pickup position.
 *   - boolean goToDropOff(): Moves the arm to the drop-off position.
 *   - void travelMode(): Puts the arm in travel mode.
 *   - void wristUp(): Moves the wrist up.
 *   - void wristTo(double wristToMove): Moves the wrist to the specified position.
 *   - void wristDown(): Moves the wrist down.
 *   - void open(): Opens the claw.
 *   - void close(): Closes the claw.
 *   - void toggleOpen(): Toggles the claw between open and closed states.
 *   - void toggleRoll(): Toggles the roll servo position.
 *   - void rollPositive(): Increases the roll servo position.
 *   - void rollNegative(): Decreases the roll servo position.
 *   - String toString(): Provides a string representation of the arm's state.
 *   - void changeOffset(int delta): Changes the encoder offset for motor position correction.
 */

package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.Locale;

public class Arm {
    // SUBSYSTEM ASSETS
    private final Servo wristServo;
    private final Servo openServo;
    private final Servo rollServo;
    private final DcMotor motor;
    private LinearOpMode opMode;

    // STATE VARIABLES
    private double wristAng = Constants.WRIST_ON_GROUND;
    private double rollPos = Constants.ROLL_MAX;
    private boolean isOpen = false;
    private int offset = 0;
    public enum RunState {
        GOTO_DROPOFF,
        GOTO_GROUND,
        GOTO_LOW,
        NONE
    }


    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
        this.motor = opMode.hardwareMap.get(DcMotor.class, Constants.ARM_MOTOR_NAME);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServo = opMode.hardwareMap.get(Servo.class, Constants.WRIST_SERVO_NAME);
        openServo = opMode.hardwareMap.get(Servo.class, Constants.OPEN_SERVO_NAME);
        rollServo = opMode.hardwareMap.get(Servo.class, Constants.ROLL_SERVO_NAME);
        wristServo.setPosition(wristAng);
        openServo.setPosition(Constants.CLAW_CLOSED_POS);
        rollServo.setPosition(rollPos);
    }

    public void move(double shoulderRot){
        opMode.telemetry.addData("Arm", motor.getCurrentPosition());
        if(opMode.telemetry != null) {
            opMode.telemetry.addData("Shoulder Pos", motor.getCurrentPosition());
            opMode.telemetry.addData("Shoulder Power", motor.getPower());
            opMode.telemetry.addData("Shoulder Offset", offset);
        }

        if((shoulderRot < 0 && motor.getCurrentPosition() + offset >= Constants.ARM_UP_POSITION) ||
                (shoulderRot > 0 && motor.getCurrentPosition() + offset <= Constants.ARM_DOWN_POSITION))
            motor.setPower(shoulderRot * Constants.ARM_MOTOR_STRENGTH);
        else
            motor.setPower(0);
    }

    public boolean goToPickUp() {
        // ongoing motion check... should it continue moving?
        if(motor.getCurrentPosition() + offset <= Constants.ARM_DOWN_POSITION)
            motor.setPower(Constants.ARM_MOTOR_STRENGTH);
        else return true;
        // Encoder values go down as arm goes up
        // -1400 + x >= -400
        // offset corrects encoder "walk"
        if(motor.getCurrentPosition() + offset >= -1350){
            rollServo.setPosition(Constants.ROLL_MAX);
            rollPos = Constants.ROLL_MAX;
            wristServo.setPosition(Constants.WRIST_ON_GROUND);
            wristAng = Constants.WRIST_ON_GROUND;
        }
        open();
        return false;
    }

    public boolean goToDropOff() {
        close();
        if(motor.getCurrentPosition() + offset >= Constants.ARM_UP_POSITION) {
            motor.setPower(-Constants.ARM_MOTOR_STRENGTH);
        } else return true;
        // Encoder values go down as arm goes up
        if(motor.getCurrentPosition() + offset <= -400){
            rollServo.setPosition(Constants.ROLL_MIN);
            rollPos = Constants.ROLL_MIN;
            wristServo.setPosition(Constants.WRIST_ON_WALL);
            wristAng = Constants.WRIST_ON_WALL;
        }
        return false;
    }

    public void travelMode() {
        close();
        wristServo.setPosition(0);
        wristAng = 0;
    }

    public void wristUp() {
        wristAng -= Constants.WRIST_INC;
        if (wristAng <= Constants.WRIST_MIN) {
            wristAng = Constants.WRIST_MIN;
        }
        wristServo.setPosition(wristAng);
    }

    public void wristTo(double wristToMove) {
        wristToMove = Range.clip(wristToMove, Constants.WRIST_MIN, Constants.WRIST_MAX);
        wristServo.setPosition(wristToMove);
        wristAng = wristToMove;
    }

    public void wristDown() {
        wristAng += Constants.WRIST_INC;
        wristAng = Range.clip(wristAng, Constants.WRIST_MIN, Constants.WRIST_MAX);
        wristServo.setPosition(wristAng);
    }

    public void open() {
        openServo.setPosition(Constants.CLAW_OPEN_POS);
        isOpen = true;
    }
    public void close() {
        openServo.setPosition(Constants.CLAW_CLOSED_POS);
        isOpen = false;
    }

    public void toggleOpen() {
        if(isOpen) {
            close();
        } else {
            open();
        }
    }

    public void toggleRoll() {
        if(rollServo.getPosition() >= .5){
            rollServo.setPosition(Constants.ROLL_MIN);
            rollPos = Constants.ROLL_MIN;
        } else{
            rollServo.setPosition(Constants.ROLL_MAX);
            rollPos = Constants.ROLL_MAX;
        }
    }

    public void rollPositive() {
        rollPos +=  Constants.ROLL_INC;
        if(rollPos > Constants.ROLL_MAX)
            rollPos = Constants.ROLL_MAX;

        rollServo.setPosition(rollPos);
    }

    public void rollNegative() {
        rollPos -=  Constants.ROLL_INC;
        if(rollPos < Constants.ROLL_MIN)
            rollPos = Constants.ROLL_MIN;

        rollServo.setPosition(rollPos);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "OPR: (%f, %f, %f)", openServo.getPosition(), wristServo.getPosition(), rollServo.getPosition());
    }

    public void changeOffset(int delta) {
        offset += delta;
    }
}
