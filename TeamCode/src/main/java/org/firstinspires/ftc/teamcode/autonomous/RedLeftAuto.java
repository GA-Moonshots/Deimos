package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        drive.gotoBackDistance(0.12, 24, 4);

        drive.faceTheProp(0.3);
        stop();
        arm.open();
        sleep(100);

        // lift claw to get out of the way
        arm.travelMode();

        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method to avoid running over the prop.
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 2);

        // straighten out
        drive.goToZero();

        // back to wall the rest of the way
        drive.gotoBackDistance(4);
        drive.autonomouslyMove(0.2, 5, MecanumDrive.HowToMove.LEFT, 3);
        drive.autonomouslyDriveByTime(-0.5 , 0.0, 0.0,2.65);
        drive.autonomouslyMove(0.3, 5, MecanumDrive.HowToMove.RIGHT, 5);

    }
}
