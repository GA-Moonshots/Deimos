package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue - LEFT")
public class LeftBlueAuto extends LinearOpMode {
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
        waitForStart();

        // approach and face the prop
        drive.fwdFromWall(20);
        drive.faceTheProp(0.3);
        // go robot centric driving
        drive.isFieldCentric = false;
        // nudge forward
        drive.nudge(-0.2);
        sleep(100);
        // lowers claw and drops pixel
        arm.goToPickUp();
        sleep(1000);

        arm.travelMode();

        // get back
        drive.nudge(0.3);

        drive.turnToZero();

        // GET TO PARK
        drive.backUpToWall(4);
        drive.strafeUntilWall(-0.3);

        // DONE: CLEAN UP
        drive.stop();
        this.terminateOpModeNow();

    }
}
