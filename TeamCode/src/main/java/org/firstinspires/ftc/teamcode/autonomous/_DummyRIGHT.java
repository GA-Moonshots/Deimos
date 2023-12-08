package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = ".Dummy RIGHT")
public class _DummyRIGHT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(this);

        drive.makeRobotCentric();

        waitForStart();
        drive.autonomouslyDriveByTime(0.0, -0.3, 0.0, 5);


        drive.stop();
    }
}
