package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline {
    public static final double TAGSIZE = 0.166;
    public static final double CAMERA_FOCAL_LENGTH_X = 578.272;
    public static final double CAMERA_FOCAL_LENGTH_Y = 578.272;
    public static final double CAMERA_X_OFFSET = 402.145;
    public static final double CAMERA_Y_OFFSET = 221.506;
    public static final double CAMERA_YZ_SHEAR = 0; // Can adjust by the 30 degrees of the center stage backdrop


    private Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    /**
     * the camera intrinsic matrix.
     * https://ksimek.github.io/2013/08/13/intrinsic/
     *
     * ---------------
     * | fx   0   cx |
     * | 0    fy  cy |
     * | 0    yz  1  |
     * ---------------
     */
    private Mat cameraMatrix;

    // SYNCHRONIZED OBJECTS
    private final Object decimationSync = new Object();
    private boolean needToSetDecimation = false;
    private long nativeApriltagPtr;
    private float decimation;

    private volatile ArrayList<AprilTagDetection> detections = new ArrayList<>();

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    public class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

    public AprilTagPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, CAMERA_FOCAL_LENGTH_X);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, CAMERA_X_OFFSET);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,CAMERA_FOCAL_LENGTH_Y);
        cameraMatrix.put(1,2,CAMERA_Y_OFFSET);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,CAMERA_YZ_SHEAR);
        cameraMatrix.put(2,2,1);

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat gray = new Mat();
        // Convert to greyscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }

            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, gray, TAGSIZE,
                    CAMERA_FOCAL_LENGTH_X, CAMERA_FOCAL_LENGTH_Y, CAMERA_X_OFFSET, CAMERA_Y_OFFSET);

            for(AprilTagDetection detection : detections) {
                Pose pose = new Pose();
                pose.tvec.put(0,0, detection.pose.x);
                pose.tvec.put(1,0, detection.pose.y);
                pose.tvec.put(2,0, detection.pose.z);

                Mat Rot = new Mat(3, 3, CvType.CV_32F);

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        //Rot.put(i,j, detection.pose.R.get(i,j));
                    }
                }
            }
        }
        telemetry.update();

        return gray;
    }
}
