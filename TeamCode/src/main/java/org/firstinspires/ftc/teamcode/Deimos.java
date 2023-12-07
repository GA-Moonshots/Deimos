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
            //telemetry.addData("UPS", 1 / (timer.seconds() - lastTime));
            telemetry.addData("Camera:", drive.camera.getStatus());
            //lastTime = timer.seconds();

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

        // Y BUTTON : available

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

            drive.drive(forward * speedMod, strafe, turn);
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

        if(armState == Arm.RunState.GOTO_DROPOFF) {
            if (arm.goToDropOff())
                armState = Arm.RunState.NONE;
            return;
        } else if(armState == Arm.RunState.GOTO_GROUND) {
            if (arm.goToPickUp())
                armState = Arm.RunState.NONE;
            return;
        }
        if(gamepad2.right_trigger >= Constants.INPUT_THRESHOLD){
            launcher.release();
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
