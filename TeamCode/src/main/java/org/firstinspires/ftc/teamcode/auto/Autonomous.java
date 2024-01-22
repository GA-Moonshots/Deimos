package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private boolean isLeft = false;
    private boolean isRed = false;
    private boolean goForBoard = false;
    private boolean gotoOpposite = false;

    private MecanumDrive drive;
    private Arm arm;

    private Camera.AprilTagToAlign align;

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
        arm.wristTo(Constants.WRIST_ON_GROUND);
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
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 2);
        drive.goToZeroAngle();
        if(gotoOpposite) {
            drive.gotoBackDistance(0.15, 8, 3.5);
            drive.autonomouslyDriveByTime(0.0, -0.2, 0.0, 4);
            if(isRed)
                drive.gotoRightDistance(20);
            else
                drive.gotoLeftDistance(20);
        } else {
            drive.gotoBackDistance(15);
            stop();
            double targetDistance;
            if(isRed)
                targetDistance = drive.leftDistance.doubleCheckDistance();
            else
                targetDistance = drive.rightDistance.doubleCheckDistance();
            if(align != Camera.AprilTagToAlign.LEFT) {
                // Store the current distance for the next piece
                // Move left before going forward
                if(isRed)
                    drive.gotoLeftDistance(10);
                else
                    drive.gotoRightDistance(10);
            } else {
                if(isRed) {
                    drive.gotoLeftDistance(8);
                } else {
                    drive.gotoRightDistance(8);
                }
            }
            drive.autonomouslyDriveByTime(-0.2, 0.0, 0.0, 5);
            if(align != Camera.AprilTagToAlign.LEFT) {
                if(isRed)
                    drive.gotoLeftDistance(targetDistance);
                else
                    drive.gotoRightDistance(targetDistance);
            }
            stop();
            drive.autonomouslyDriveByTime(0.0, isRed ? 0.4 : -0.4, 0.0, 4);
        }
    }

    public void parkInside() {
        // park for 5 points
        drive.autonomouslyMove(0.3, 5,
                //boolean statement ? if true : if false
                isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT,
                4.75);
    }

    public void parkOutside() {
        if(gotoOpposite)
            drive.autonomouslyDriveByTime(-0.3, 0.0, 0.0, 3.75);
        drive.goToZeroAngle();
        if(isRed)
            drive.gotoRightDistance(4);
        else
            drive.gotoLeftDistance(4);
        drive.autonomouslyDriveByTime(0.1, 0.0, 0.0, 1);
    }
}
