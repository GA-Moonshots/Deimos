package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red - RIGHT")
public class Autonomous extends LinearOpMode {
    private boolean isLeft = false;
    private boolean isRed = false;
    private boolean goForBoard = false;

    private MecanumDrive drive;
    private Arm arm;

    private MecanumDrive.AprilTagToAlign align;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(this);
        arm = new Arm(this);

        while(opModeInInit()) {
            // Set side of field (left/right, red/blue)
            if(gamepad1.dpad_left)
                isLeft = true;
            else if(gamepad1.dpad_right)
                isLeft = false;

            if(gamepad1.x)
                isRed = false;
            else if(gamepad1.b && !gamepad1.start)
                isRed = true;

            // Should we go for 45 points instead of 25?
            if(gamepad1.y)
                goForBoard = true;
            else if(gamepad1.a && !gamepad1.start)
                goForBoard = false;

            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue", isLeft ? "Left" : "Right");
            telemetry.addData("Target Points", "%s", goForBoard ? "45" : "25");

            // approach the prop
            drive.gotoBackDistance(0.115, 24, 4);

            align = drive.faceTheProp(0.3, 2);
            stop();
            arm.open();
            sleep(100);

            // lift claw to get out of the way
            arm.travelMode();

            if((isLeft && !isRed) || (!isLeft && isRed)) {
                near();
            } else {
                far();
            }
            // align

            if((isLeft && !isRed) || (!isLeft && isRed)) {
                parkInside();
            } else {
                parkOutside();
            }
        }
    }

    public void near() {
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method.
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 2);

        // straighten out
        drive.goToZero();

        // back up to the wall the rest of the way
        drive.gotoBackDistance(4);

        // park for 5 points
        drive.autonomouslyMove(0.3, 5,
                isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT,
                4.75);
    }

    public void far() {
        if(align == MecanumDrive.AprilTagToAlign.CENTER) {
            // Move left before going forward
        } else {
            // If we are facing left we are already facing the correct way
            double turn = (align == MecanumDrive.AprilTagToAlign.LEFT) ? 0.0 : -0.4;
            // Simply go straight ahead
            drive.autonomouslyDriveByTime(-0.5, 0.0, 0.0, 1.25);
            // Cheat and turn at the same time
            drive.autonomouslyDriveByTime(0.0, 0.5, turn, 3.5);
        }
        drive.gotoAngle(90);

    }

    public void parkInside() {

    }

    public void parkOutside() {

    }
}
