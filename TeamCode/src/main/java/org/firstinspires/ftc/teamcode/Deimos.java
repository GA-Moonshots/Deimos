/*
 * Deimos Class - FTC Robot TeleOp OpMode
 *
 * This class represents the TeleOp OpMode for a FTC (First Tech Challenge) robot named "Deimos."
 * The TeleOp mode allows for manual control of various robot subsystems, including the MecanumDrive,
 * Arm, Elevator, and Launcher.
 *
 * Author: [everyone who worked on it]
 * Last Modified: 12/8/2023 1:07pm
 * Version: [self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - Deimos
 *       - LinearOpMode
 *       - MecanumDrive
 *       - Arm
 *       - Elevator
 *       - Launcher
 *
 * Fields:
 *   - Arm arm: Arm instance for controlling the robot's arm subsystem.
 *   - MecanumDrive drive: MecanumDrive instance for controlling the robot's mecanum drive system.
 *   - Elevator elevator: Elevator instance for controlling the robot's elevator subsystem.
 *   - Launcher launcher: Launcher instance for controlling the robot's launcher subsystem.
 *   - double lastTime: Variable to store the last time value for calculating update rates.
 *   - ElapsedTime timer: ElapsedTime instance for tracking time intervals.
 *   - MecanumDrive.AprilTagToAlign align: Variable to store the April Tag alignment status.
 *   - Arm.RunState armState: Variable to store the current state of the arm.
 *   - boolean gp1aPressed: Toggle variable to avoid double-pressing the 'A' button on gamepad 1.
 *   - boolean gp1bPressed: Toggle variable to avoid double-pressing the 'B' button on gamepad 1.
 *   - boolean gp2aPressed: Toggle variable to avoid double-pressing the 'A' button on gamepad 2.
 *   - boolean gp2bPressed: Toggle variable to avoid double-pressing the 'B' button on gamepad 2.
 *   - boolean gp2xPressed: Toggle variable to avoid double-pressing the 'X' button on gamepad 2.
 *   - boolean gp2yPressed: Toggle variable to avoid double-pressing the 'Y' button on gamepad 2.
 *   - boolean gp2rbPressed: Toggle variable to avoid double-pressing the right bumper on gamepad 2.
 *
 * Methods:
 *   - runOpMode(): Entry point for the TeleOp OpMode, where manual control is provided to the driver.
 *   - driver1Inputs(): Method handling inputs from gamepad 1, mainly for drivetrain control.
 *   - driver2Inputs(): Method handling inputs from gamepad 2, including arm, elevator, and launcher controls.
 *
 * Note: This OpMode includes two driver control methods, one for each gamepad, to provide clarity and organization.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Elevator;
import org.firstinspires.ftc.teamcode.systems.Launcher;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@TeleOp(name="Deimos")
public class Deimos extends LinearOpMode {

    // SUBSYSTEMS
    private Arm arm;
    private MecanumDrive drive;
    private Elevator elevator;
    private Launcher launcher;

    // INSTANCE VARIABLES
    private double lastTime = 0.0d;
    private final ElapsedTime timer = new ElapsedTime();
    private MecanumDrive.AprilTagToAlign align = MecanumDrive.AprilTagToAlign.NONE;
    private Arm.RunState armState = Arm.RunState.NONE;

    // CONTROLLER TOGGLES TO AVOID DOUBLE PRESSING
    private boolean gp1aPressed = false;
    private boolean gp1bPressed = false;
    private boolean gp2aPressed = false;
    private boolean gp2bPressed = false;
    private boolean gp2xPressed = false;
    private boolean gp2yPressed = false;
    private boolean gp2rbPressed = false;
    private boolean gp2lbPressed = false;
    private boolean gp2rtPressed = false;
    
    @Override
    public void runOpMode() {
        // Init (runs once)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);
        arm = new Arm(this);
        elevator = new Elevator(this);
        launcher = new Launcher(this);

        // Init Loop (runs until stop button or start button is pressed)
        while(opModeInInit()) {

            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.addData("Camera:", drive.camera.getStatus());
            drive.postDistanceReadouts();
            telemetry.update();
        }

        timer.reset();
        waitForStart();

        // ---MAIN EXECUTION LOOP (runs until stop is pressed) --
        while(opModeIsActive()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("UPS", 1 / (timer.seconds() - lastTime));
            telemetry.addData("Camera:", drive.camera.getStatus());
            lastTime = timer.seconds();

            // Driver 1: Responsible for drivetrain and movement
            if(opModeIsActive()) driver1Inputs();
            // Driver 2: Responsible for the subsystem attachment
            if(opModeIsActive()) driver2Inputs();

            telemetry.update();
        }
    }

    /**
     * Driver 1: Solely responsible for the control of the drivetrain;
     * this function never changes and should not be changed unless addition of a new
     * feature is required.
     */
    private void driver1Inputs() {

        // A BUTTON: toggles field-centric VS robot-centric driving
        if(gamepad1.a && !gp1aPressed && !gamepad1.start) {
            drive.toggleFieldCentric();
        }
        gp1aPressed = gamepad1.a; // this structure avoids double press

        // B BUTTON: available
        if (gamepad1.b && !gp1bPressed && !gamepad2.start) {
        }
        gp1bPressed = gamepad1.b; // this structure avoids double press

        // X BUTTON: available

        // Y BUTTON : Reset Field Centric Target
        if(gamepad1.y){
            drive.resetFieldCentricTarget();
        }


        if(gamepad1.right_trigger >= Constants.INPUT_THRESHOLD){
            launcher.release();

        }

        // DPAD: align to April Tag
        boolean dpadUpPressed = (gamepad1.dpad_up && !gamepad1.dpad_down);
        boolean dpadDownPressed = (gamepad1.dpad_down && !gamepad1.dpad_up);
        boolean dpadLeftPressed = (gamepad1.dpad_left && !gamepad1.dpad_right);
        boolean dpadRightPressed = (gamepad1.dpad_right && !gamepad1.dpad_left);
        if(dpadLeftPressed && !(dpadUpPressed || dpadDownPressed || dpadRightPressed)) {
            align = MecanumDrive.AprilTagToAlign.LEFT;
        } else if(dpadRightPressed && !(dpadUpPressed || dpadDownPressed || dpadLeftPressed)) {
            align = MecanumDrive.AprilTagToAlign.RIGHT;
        } else if(dpadUpPressed && !(dpadLeftPressed || dpadDownPressed || dpadRightPressed)) {
            align = MecanumDrive.AprilTagToAlign.CENTER;
        } else if(dpadDownPressed && !(dpadUpPressed || dpadLeftPressed || dpadRightPressed)) {
            align = MecanumDrive.AprilTagToAlign.NONE;
        }

        // JOYSTICK: motion control - left stick strafes / right stick rotates
        // is the pilot denied control of the robot while we line up to an April tag?
        if(align != MecanumDrive.AprilTagToAlign.NONE) {
            //if(!drive.alignToAprilTag(align)) {
             //   align = MecanumDrive.AprilTagToAlign.NONE;
            //}
        // listen to driver controls
        } else {
            telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
            double speedMod = gamepad1.right_bumper ? 0.5 : 1; // slow mode
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // DEAD-ZONES
            if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
            if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
            if (Math.abs(turn) <= Constants.INPUT_THRESHOLD) turn = 0.0d;

            drive.drive(forward * speedMod, strafe * speedMod, turn * speedMod);
        }

    }

    /**
     * Driver 2: responsible for any subsystem attachments we may have.
     * This function's implementation changes quickly and rapidly every year.
     */
    private void driver2Inputs() {
        // A BUTTON: grip game object for traveling
        if(gamepad2.a && !gamepad2.start){
            arm.travelMode();
        }

        // B BUTTON: toggle claw
        if(gamepad2.b && !gp2bPressed && !gamepad2.start) {
            arm.toggleOpen();
            telemetry.addData("Hand open", gamepad2.b);
        }
        gp2bPressed = gamepad2.b; // this structure avoids double press

        // X BUTTON: position arm for pickup
        if(gamepad2.x && !gp2xPressed && armState != Arm.RunState.GOTO_GROUND)
            armState = Arm.RunState.GOTO_GROUND;
        else if(gamepad2.x && !gp2xPressed)
            armState = Arm.RunState.NONE;
        gp2xPressed = gamepad2.x;

        // Y BUTTON: position arm for drop-off
        if(gamepad2.y && !gp2yPressed && armState != Arm.RunState.GOTO_DROPOFF)
            armState = Arm.RunState.GOTO_DROPOFF;
        else if(gamepad2.y && !gp2yPressed)
            armState = Arm.RunState.NONE;
        gp2yPressed = gamepad2.y;

        if(gamepad2.right_trigger >= Constants.INPUT_THRESHOLD && !gp2rtPressed && armState != Arm.RunState.GOTO_DROPOFF) {
            armState = Arm.RunState.GOTO_LOW;
        } else if(gamepad2.right_trigger >= Constants.INPUT_THRESHOLD && !gp2rtPressed) {
            armState = Arm.RunState.NONE;
        }
        gp2rtPressed = gamepad2.right_trigger >= Constants.INPUT_THRESHOLD;


        if(armState == Arm.RunState.GOTO_DROPOFF) {
            if (arm.goToDropOff())
                armState = Arm.RunState.NONE;
            return;
        } else if(armState == Arm.RunState.GOTO_GROUND) {
            if (arm.goToPickUp())
                armState = Arm.RunState.NONE;
            return;
        } else if(armState == Arm.RunState.GOTO_LOW) {
            //arm.
        }


        if(gamepad2.right_bumper && !gp2rbPressed) {
            elevator.toggleLock();
        }
        gp2rbPressed = gamepad2.right_bumper;


        // DPAD VERTICAL: wrist
        if(gamepad2.dpad_up && !gamepad2.dpad_down) {
            arm.wristUp();
        } else if(gamepad2.dpad_down && !gamepad2.dpad_up) {
            arm.wristDown();
        }
        // DPAD HORIZONTAL: roll
        if(gamepad2.dpad_left && !gamepad2.dpad_right) {
            arm.rollNegative();
        } else if(gamepad2.dpad_right && !gamepad2.dpad_left) {
            arm.rollPositive();
        }

        // JOYSTICKS: lift and lower arm
        double armRotate = gamepad2.left_stick_y;
        double elevatorStr = gamepad2.right_stick_y;
        // DEAD-ZONE
        if(Math.abs(armRotate) <= Constants.INPUT_THRESHOLD) armRotate = 0;
        // Offsets
        if(Math.abs(gamepad2.left_trigger) >= Constants.INPUT_THRESHOLD)
            arm.changeOffset(5);
        else if(gamepad2.left_bumper)
            arm.changeOffset(-5);
        arm.move(armRotate);

        if(Math.abs(elevatorStr) <= Constants.INPUT_THRESHOLD) elevatorStr = 0;
        elevator.move(elevatorStr);
    }

}
