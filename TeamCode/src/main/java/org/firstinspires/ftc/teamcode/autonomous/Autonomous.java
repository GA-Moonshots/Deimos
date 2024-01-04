package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private boolean isLeft = false;
    private boolean isRed = false;
    private boolean goForBoard = false;
    private boolean gotoInside = false;

    private MecanumDrive drive;
    private Arm arm;

    private MecanumDrive.AprilTagToAlign align;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(this);
        arm = new Arm(this);

        while(opModeInInit()) {
            // Set side of field (left/right, red/blue)
            //Do we start in fieldcentric? if not then why?
            if (gamepad1.dpad_left)
                isLeft = true;
            else if (gamepad1.dpad_right)
                isLeft = false;

            if (gamepad1.x)
                isRed = false;
            else if (gamepad1.b && !gamepad1.start)
                isRed = true;

            // Should we go for 45 points instead of 25?
            if (gamepad1.y)
                goForBoard = true;
            else if (gamepad1.a && !gamepad1.start)
                goForBoard = false;

            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue", isLeft ? "Left" : "Right");
            telemetry.addData("Target Points", "%s", goForBoard ? "45" : "25");
            telemetry.update();
        }


        //force field centric
        drive.makeFieldCentric();
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

    public void near() {
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method.
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 2);

        // straighten out
        drive.goToZeroAngle();

        // back up to the wall the rest of the way
        drive.gotoBackDistance(4);


    }

    public void far() {
        // If we are facing left we are already facing the correct way
        double turn = 0.4d;

        if(align == MecanumDrive.AprilTagToAlign.LEFT && isRed) {
            turn = 0.0d;
        } else if(align == MecanumDrive.AprilTagToAlign.RIGHT && !isRed) {
            turn = 0.0d;
        }

        double multiplier = isRed ? 1 : -1;

        if(align == MecanumDrive.AprilTagToAlign.CENTER) {
            // Store the current distance for the next piece
            double targetLeftDistance = drive.leftDistance.getDistance();
            // Move left before going forward
            drive.gotoLeftDistance(10);
            drive.gotoLeftDistance(targetLeftDistance);
        }

        drive.autonomouslyDriveByTime(-0.5, 0.0, 0.0, 1.25);
        // Cheat and turn at the same time
        drive.autonomouslyDriveByTime(0.0, multiplier * 0.5, multiplier * turn, 3.5);

        drive.gotoAngle(90 * multiplier);
    }

    public void parkInside() {
        // park for 5 points
        drive.autonomouslyMove(0.3, 5,
                //boolean statement ? is boolean true : is if false
                isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT,
                4.75);
    }

    public void parkOutside() {
        drive.makeRobotCentric();
        drive.gotoBackDistance(5);
        drive.makeFieldCentric();
        drive.autonomouslyDriveByTime(0.1, 0.0, 0.0, 2);
    }
}
