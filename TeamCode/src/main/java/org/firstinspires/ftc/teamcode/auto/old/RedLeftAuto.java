package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red - LEFT")
public class RedLeftAuto extends LinearOpMode {
    // SUBSYSTEMS
    private MecanumDrive drive;
    private Arm arm;

    private enum TargetPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);
        arm = new Arm(this);

        while(opModeInInit())
            drive.postDistanceReadouts();

        waitForStart();

        // approach the prop
        drive.gotoBackDistance(0.115, 24, 4);

        Camera.AprilTagToAlign align = drive.faceTheProp(0.3, 2);
        stop();
        arm.open();
        sleep(100);

        // lift claw to get out of the way
        arm.travelMode();

        if(align == Camera.AprilTagToAlign.CENTER) {
            // Move left before going forward
            drive.gotoLeftDistance(10);
        } else {
            // If we are facing left we are already facing the correct way
            double turn = (align == Camera.AprilTagToAlign.LEFT) ? 0.0 : -0.4;
            // Simply go straight ahead
            drive.autonomouslyDriveByTime(-0.5, 0.0, 0.0, 1.25);
            drive.autonomouslyDriveByTime(0.0, 0.5, turn, 3.5);
        }
        drive.gotoAngle(90);

        // Raise the arm somewhere in here?

        terminateOpModeNow();
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method to avoid running over the prop.
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 1.9);

        // back to wall the rest of the way
        drive.gotoBackDistance(4);
        drive.autonomouslyMove(0.2, 5, MecanumDrive.HowToMove.LEFT, 3);
        drive.autonomouslyDriveByTime(-0.5 , 0.0, 0.0,2.5);
        drive.goToZeroAngle();
        drive.autonomouslyMove(0.3, 5, MecanumDrive.HowToMove.RIGHT, 5);
        drive.autonomouslyDriveByTime(.2,0.0,0.0,1.0);

    }
}
