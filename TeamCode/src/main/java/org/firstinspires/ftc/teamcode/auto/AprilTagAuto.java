package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.annotation.Target;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AprilTagAuto")

public class AprilTagAuto extends LinearOpMode {

    private MecanumDrive drive;
    private Arm arm;
    private Camera.AprilTagToAlign align = Camera.AprilTagToAlign.LEFT;
    private List<AprilTagDetection> detections;



    final double DESIRED_DISTANCE = 8.0;


    @Override public void runOpMode(){
        drive = new MecanumDrive(this);
        arm = new Arm(this);


        AprilTagDetection target = null;

        // we are right in front of the board for testing
        waitForStart();

        while(opModeIsActive()){

            target = fetchTarget();


            // pull our desired target from detections

            if(target != null){
                updateTargetTelemetry(target);

                // fix our delta x value for alignment
                while(opModeIsActive() && target.ftcPose.x < -2){
                    drive.makeRobotCentric();
                    drive.drive(0.0,0.2,0.0);
                }
                drive.stop();

            }
            else{
                System.out.println("lmao sorry");
            }






        }



    }

    public void updateTargetTelemetry(AprilTagDetection target) {
        telemetry.addData("target x", target.ftcPose.x);
        telemetry.addData("target y", target.ftcPose.y);
        telemetry.addData("target range", target.ftcPose.range);
        telemetry.addData("# of detects", detections.size());
        telemetry.update();
        sleep(2);

    }

    public AprilTagDetection fetchTarget() {
        detections = drive.camera.getDetections();

        for(AprilTagDetection april : detections){
            if(april.metadata.name.toLowerCase().indexOf(align.name().toLowerCase()) != -1)
                 return april;
        }
        return null;

    }


}
