package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Right Red Autonomous")
public class RightRedAuto extends LinearOpMode {
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
        arm = new Arm(hardwareMap, null);

        waitForStart();

        // approach the prop
        drive.fwdFromWall(20);

        drive.faceTheProp();

        // go robot centric driving
        drive.toggleFieldCentric();

        // nudge forward
        drive.nudge(-0.2);
        sleep(100);

        // lowers claw and drops pixel
        arm.goToPickUp();
        sleep(1000);

        // move claw out of the pixel's way
        arm.travelMode();

        // get back
        drive.nudge(0.3);

        drive.turnToZero();

        // GET TO PARK
        drive.backUpToWall(4);
        drive.strafeUntilWall(0.3);

        drive.stop();
        drive.camera.shutdown();
        this.terminateOpModeNow();
    }
}
