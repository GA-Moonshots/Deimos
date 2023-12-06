package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue - RIGHT")
public class BlueRightAuto extends LinearOpMode {
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

        // approach prop
        drive.gotoBackDistance(0.2, 22, 3);
        drive.faceTheProp(0.3, MecanumDrive.HowToMove.ROTATE_LEFT, 8);

        // TODO: support completely robot-centric autonomous
        drive.makeRobotCentric();

        // nudge the pixel forward and hope for the best
        // Forward is negative Mr. A...
        drive.autonomouslyDriveByTime(-0.2, 0.0, 0.0, 0.5);

        // lift the claw x2
        arm.goToPickUp();
        arm.open();
        sleep(500); // Ensure the arm opens

        arm.travelMode();
        drive.autonomouslyDriveByTime(0.2, 0.0, 0.0, 0.5);
        drive.goToZero();

        // TODO: support completely robot-centric autonomous
        drive.makeFieldCentric();

        // JOURNEY AROUND THE HUNKS OF METAL TO PARK
        drive.gotoBackDistance(4);
        drive.autonomouslyMove(0.2, 5, MecanumDrive.HowToMove.RIGHT, 5);
        drive.gotoBackDistance(50, 5);
        drive.autonomouslyMove(0.7, 5, MecanumDrive.HowToMove.LEFT, 5);

    }
}
