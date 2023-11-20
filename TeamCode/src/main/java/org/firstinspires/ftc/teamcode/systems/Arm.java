package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public class Arm {
    // CONSTANTS
    public static final int UP_POSITION = -1400;
    public static final int DOWN_POSITION = 0;
    public static final double MOTOR_STRENGTH = 0.9;
    public static final double WRIST_ON_WALL =  0.6;

    public static final double OPEN_POS = 0.5;
    public static final double CLOSED_POS = 0.9;

    public static final double WRIST_INC = 0.01;
    public static final double WRIST_MAX = 0.8;
    public static final double WRIST_ON_GROUND = 0.36;

    public static final double WRIST_MIN = 0.0;

    public static final double ROLL_MAX = 0.72;
    public static final double ROLL_MIN = 0;
    public static final double ROLL_INC = 0.03;

    // STATE VARIABLES
    private double wristAng = WRIST_MIN;
    private double rollPos = ROLL_MAX;
    private boolean isOpen = false;
    private int offset = 0;

    // SUBSYSTEM ASSETS
    private final Servo wristServo;
    private final Servo openServo;
    private final Servo rollServo;

    private final Telemetry telemetry;
    private final DcMotor motor;


    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.motor = hardwareMap.get(DcMotor.class, "arm");
        this.telemetry = telemetry;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServo = hardwareMap.get(Servo.class, "wrist");
        openServo = hardwareMap.get(Servo.class, "open");
        rollServo = hardwareMap.get(Servo.class, "roll");
        wristServo.setPosition(wristAng);
        openServo.setPosition(1);
        rollServo.setPosition(rollPos);
    }

    public void move(double shoulderRot){
        if(telemetry != null) {
            telemetry.addData("Shoulder Pos", motor.getCurrentPosition());
            telemetry.addData("Shoulder Power", motor.getPower());
            telemetry.addData("Shoulder Offset", offset);
        }

        if((shoulderRot < 0 && motor.getCurrentPosition() + offset >= UP_POSITION) || (shoulderRot > 0 && motor.getCurrentPosition() + offset <= DOWN_POSITION))
            motor.setPower(shoulderRot * MOTOR_STRENGTH);
        else
            motor.setPower(0);
    }

    public void goToPickUp() {
        // ongoing motion check... should it continue moving?
        if(motor.getCurrentPosition() + offset <= DOWN_POSITION)
            motor.setPower(MOTOR_STRENGTH);
        // Encoder values go down as arm goes up
        // -1400 + x >= -400
        // offset corrects encoder "walk"
        if(motor.getCurrentPosition() + offset >= -1350){
            rollServo.setPosition(ROLL_MAX);
            rollPos = ROLL_MAX;
            wristServo.setPosition(WRIST_MIN);
            wristAng = WRIST_MIN;
        }
        open();
    }

    public void goToDropOff() {
        close();
        if(motor.getCurrentPosition() + offset >= UP_POSITION)
            motor.setPower(-MOTOR_STRENGTH);
        // Encoder values go down as arm goes up
        if(motor.getCurrentPosition() + offset <= -400){
            rollServo.setPosition(ROLL_MIN);
            rollPos = ROLL_MIN;
            wristServo.setPosition(WRIST_ON_WALL);
            wristAng = WRIST_ON_WALL;
        }
    }

    public void travelMode() {
        close();
        wristServo.setPosition(0);
        wristAng = 0;
    }

    public void wristUp() {
        wristAng -= WRIST_INC;
        if (wristAng <= WRIST_MIN) {
            wristAng = WRIST_MIN;
        }
        wristServo.setPosition(wristAng);
    }

    public void wristTo(double wristToMove) {
        wristToMove = Range.clip(wristToMove, WRIST_MIN, WRIST_MAX);
        wristServo.setPosition(wristToMove);
        wristAng = wristToMove;
    }

    public void wristDown() {
        wristAng += WRIST_INC;
        wristAng = Range.clip(wristAng, WRIST_MIN, WRIST_MAX);
        wristServo.setPosition(wristAng);
    }

    public void open() {
        openServo.setPosition(OPEN_POS);
        isOpen = true;
    }
    public void close() {
        openServo.setPosition(CLOSED_POS);
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
            rollServo.setPosition(ROLL_MIN);
            rollPos = ROLL_MIN;
        } else{
            rollServo.setPosition(ROLL_MAX);
            rollPos = ROLL_MAX;
        }
    }

    public void rollPositive() {
        rollPos +=  ROLL_INC;
        if(rollPos > ROLL_MAX)
            rollPos = ROLL_MAX;

        rollServo.setPosition(rollPos);
    }

    public void rollNegative() {
        rollPos -=  ROLL_INC;
        if(rollPos < ROLL_MIN)
            rollPos = ROLL_MIN;

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
