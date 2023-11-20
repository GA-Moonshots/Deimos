package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@TeleOp(name="Deimos")
public class Deimos extends LinearOpMode {

    // SUBSYSTEMS
    private Arm arm;
    private MecanumDrive driveyMcDriveDriveDriverson;

    // INSTANCE VARIABLES
    private double lastTime = 0.0d;
    private final ElapsedTime timeyMcTimeTimerTimerson = new ElapsedTime();
    private boolean gp1aPressed = false;
    private boolean gp2aPressed = false;
    private boolean gp2bPressed = false;
    private Drivetrain.AprilTagToAlign align = Drivetrain.AprilTagToAlign.NONE;
    
    @Override
    public void runOpMode() {
        // Init (runs once)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveyMcDriveDriveDriverson = new MecanumDrive(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);

        // Init Loop (runs until stop button or start button is pressed)
        while(opModeInInit()) {

            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
        }
        // Start (runs once)
        telemetry.addData("Status", "Started");
        telemetry.update();

        timeyMcTimeTimerTimerson.reset();

        // MAIN EXECUTION LOOP (runs until stop is pressed)
        while(opModeIsActive()) {

            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("UPS", 1 / (timeyMcTimeTimerTimerson.seconds() - lastTime));
            telemetry.addData("zAngle", driveyMcDriveDriveDriverson.getIMU().getZAngle());
            lastTime = timeyMcTimeTimerTimerson.seconds();
            //
            // Driver 1: Responsible for drivetrain and movement
            driver1Inputs();
            // Driver 2: Responsible for the subsystem attachment
            driver2Inputs();

            telemetry.update();
        }
        // STOP ALL SYSTEMS AFTER EXECUTION LOOP
        driveyMcDriveDriveDriverson.stop();
    }

    /**
     * Driver 1: Solely responsible for the control of the drivetrain;
     * this function never changes and should not be changed unless addition of a new
     * feature is required.
     */
    private void driver1Inputs() {

        // A BUTTON: toggles field-centric VS robot-centric driving
        boolean aDown = gamepad1.a && !gp1aPressed && !gamepad1.start;
        if(aDown) {
            driveyMcDriveDriveDriverson.toggleFieldCentric();
        }

        // B BUTTON: available
        if (gamepad1.b) {}

        // X BUTTON: testing functions
        boolean xPressed = gamepad1.x;
        if (xPressed) driveyMcDriveDriveDriverson.squareUp();

        // Y BUTTON : available

        // DPAD: align to April Tag
        boolean dpadUpPressed = (gamepad1.dpad_up && !gamepad1.dpad_down);
        boolean dpadDownPressed = (gamepad1.dpad_down && !gamepad1.dpad_up);
        boolean dpadLeftPressed = (gamepad1.dpad_left && !gamepad1.dpad_right);
        boolean dpadRightPressed = (gamepad1.dpad_right && !gamepad1.dpad_left);
        if(dpadLeftPressed && !(dpadUpPressed || dpadDownPressed || dpadRightPressed)) {
            align = Drivetrain.AprilTagToAlign.LEFT;
        } else if(dpadRightPressed && !(dpadUpPressed || dpadDownPressed || dpadLeftPressed)) {
            align = Drivetrain.AprilTagToAlign.RIGHT;
        } else if(dpadUpPressed && !(dpadLeftPressed || dpadDownPressed || dpadRightPressed)) {
            align = Drivetrain.AprilTagToAlign.CENTER;
        } else if(dpadDownPressed && !(dpadUpPressed || dpadLeftPressed || dpadRightPressed)) {
            align = Drivetrain.AprilTagToAlign.NONE;
        }

        // JOYSTICK: motion control - left stick strafes / right stick rotates
        // is the pilot denied control of the robot while we line up to an April tag?
        if(align != Drivetrain.AprilTagToAlign.NONE) {
            if(!driveyMcDriveDriveDriverson.alignToAprilTag(align)) {
                align = Drivetrain.AprilTagToAlign.NONE;
            }
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

            driveyMcDriveDriveDriverson.drive(forward * speedMod, strafe, turn);
        }

        // --END OF LOOP CHECKS--
        // Avoid double pressing the A button by storing its last state
        gp1aPressed = gamepad1.a;
    }

    /**
     * Driver 2: responsible for any subsystem attachments we may have.
     * This function's implementation changes quickly and rapidly every year.
     */
    private void driver2Inputs() {

        // A BUTTON: grip game object for traveling
        if(gamepad2.a && !gp2aPressed && !gamepad2.start){
            arm.travelMode();
        }

        // B BUTTON: toggle claw
        if(gamepad2.b && !gp2bPressed && !gamepad2.start){
            arm.toggleOpen();
            telemetry.addData("Hand open", gamepad2.b);
        }

        // X BUTTON: position arm for pickup
        if(gamepad2.x) arm.goToPickUp();

        // Y BUTTON: position arm for drop-off
        if(gamepad2.y) arm.goToDropOff();

        // DPAD: wrist
        if(gamepad2.dpad_up && !gamepad2.dpad_down) {
            arm.wristUp();
        } else if(gamepad2.dpad_down && !gamepad2.dpad_up) {
            arm.wristDown();
        }

        if(gamepad2.dpad_left && !gamepad2.dpad_right) {
            arm.rollNegative();
        } else if(gamepad2.dpad_right && !gamepad2.dpad_left) {
            arm.rollPositive();
        }

        // JOYSTICKS: lift and lower arm
        double armRotate = gamepad2.left_stick_y;
        // DEAD-ZONE
        if(Math.abs(armRotate) <= Constants.INPUT_THRESHOLD) armRotate = 0;
        // Offsets
        if(Math.abs(gamepad2.left_trigger) >= Constants.INPUT_THRESHOLD)
            arm.changeOffset(5);
        else if(gamepad2.left_bumper)
            arm.changeOffset(-5);
        arm.move(armRotate);


        // --END OF LOOP CHECKS--
        gp2aPressed = gamepad2.a;
        gp2bPressed = gamepad2.b;
        telemetry.addData("Arm", arm);
        telemetry.addData("GP2Info", String.format("(%.2f)", armRotate));
    }

}
