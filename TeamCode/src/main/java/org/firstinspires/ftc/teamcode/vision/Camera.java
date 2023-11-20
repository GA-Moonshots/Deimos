package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName name = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                .setCamera(name).addProcessor(aprilTag)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(25);
        }
        return aprilTag.getDetections();
    }

    public void shutdown() {
        //visionPortal.stopStreaming();
        visionPortal.close();
    }
}