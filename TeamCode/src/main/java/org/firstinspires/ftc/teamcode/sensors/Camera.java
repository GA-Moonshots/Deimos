/*
 * Camera Class - FTC Robot Camera Sensor
 *
 * This class represents the camera sensor subsystem on an FTC robot. It utilizes the AprilTag vision processing library
 * to detect AprilTags and obtain distance and angle information. The Camera class interfaces with the VisionPortal to access
 * camera controls and process AprilTag detections.
 *
 * Author: Michael, Vincent, Gabe, Mr, A
 * Last Modified: 12/8/2023 11:17am
 * Version: 0.1.1.0
 *
 * Class Hierarchy:
 *   - Camera
 *       - AprilTagProcessor
 *       - VisionPortal
 *
 * Subsystem Assets:
 *   - AprilTagProcessor aprilTag
 *   - VisionPortal visionPortal
 *
 * Methods:
 *   Constructors:
 *     - Camera(HardwareMap hardwareMap, Telemetry telemetry): Initializes the Camera subsystem with the AprilTag processor
 *       and VisionPortal for camera control and processing.
 *
 *   Detection Commands:
 *     - List<AprilTagDetection> getDetections(): Obtains a list of AprilTag detections from the camera.
 *
 *   Status Commands:
 *     - String getStatus(): Gets the current status of the camera, indicating whether it is streaming or not.
 *
 */

package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        WebcamName name = hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME);
        visionPortal = new VisionPortal.Builder()
                .setCamera(name)
                .addProcessor(aprilTag)
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

    public String getStatus(){
        return visionPortal.getCameraState().toString();
    }
}