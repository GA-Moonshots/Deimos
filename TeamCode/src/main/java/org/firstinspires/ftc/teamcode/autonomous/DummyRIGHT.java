package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = ".Dummy RIGHT")
public class DummyRIGHT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(this);

        drive.makeRobotCentric();

        waitForStart();

        //TODO: use a drivetrain method
        ElapsedTime rt = new ElapsedTime();
        if(opModeIsActive())
            drive.drive(0.0, 0.3, 0.0);
        while(opModeIsActive() && rt.seconds() <= 5)
            sleep(1);

        drive.stop();
    }
}
