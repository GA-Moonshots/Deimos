package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CVPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    private int conKSize = 3;
    private int cannyThresh1 = 100;
    private int cannyThresh2 = 200;

    public static class Cone {
        public String color;
        public int x;
        public int y;
        public int height;
        public int width;

        @Override
        public String toString() {
            return String.format("%s cone at (%f, %f)", color, x, y);
        }
    }

    public static class Pole {
        public enum Height {
            HIGH,
            MID,
            LOW
        }
        Height height;

        @Override
        public String toString() {
            return String.format("%s pole", height);
        }
    }

    // Since opencv runs asynchronously to the robot,
    // these MUST be declared volatile
    // to avoid memory leaks and bad decisions
    public volatile ArrayList<Cone> cones;
    public volatile ArrayList<Pole> poles;
    public CVPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        if(hsv.empty()) {
            return input;
        }
        // region draw an output image
        Mat redAreas = findColor(hsv, 175, 15); // 160-180; 0-10
        Mat yellowAreas = findColor(hsv, 30, 20); // 10-50
        Mat blueAreas = findColor(hsv, 120, 15); // 105-135
        Mat redSub = new Mat();
        Mat blueSub = new Mat();
        Mat yellowSub = new Mat();

        Core.subtract(redAreas, blueAreas, redSub);
        Core.subtract(redSub, yellowAreas, redSub);
        Core.subtract(blueAreas, redAreas, blueSub);
        Core.subtract(blueSub, yellowAreas, blueSub);
        Core.subtract(yellowAreas, redAreas, yellowSub);
        Core.subtract(yellowSub, blueAreas, yellowSub);

        Mat lines = new Mat();
        Mat redOutput = new Mat();
        Mat blueOutput = new Mat();
        Mat yellowOutput = new Mat();
        Mat rgbRed = new Mat();
        Mat rgbBlue = new Mat();
        Mat rgbYellow = new Mat();
        Imgproc.cvtColor(findLines(redSub, conKSize, cannyThresh1, cannyThresh2), rgbRed, Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(findLines(blueSub, conKSize, cannyThresh1, cannyThresh2), rgbBlue, Imgproc.COLOR_GRAY2RGB); //COLOR_GRAY2RGB
        Imgproc.cvtColor(findLines(yellowSub, conKSize, cannyThresh1, cannyThresh2), rgbYellow, Imgproc.COLOR_GRAY2RGB); //COLOR_HSV2RGB
        Core.multiply(rgbRed, new Scalar(1, 0, 0), redOutput);
        Core.multiply(rgbBlue, new Scalar(0, 0, 1), blueOutput);
        Core.multiply(rgbYellow, new Scalar(1, 1, 0), yellowOutput);
        Core.add(blueOutput, yellowOutput, lines);
        Core.add(lines, redOutput, lines);

        return lines;
        // endregion
    }

    private Mat findColor(Mat hsv, int hue, int searchingRoom) {
        Mat mask = new Mat();

        Scalar lower1;
        Scalar upper1;
        Scalar lower2 = null;
        Scalar upper2 = null;
        if (hue > searchingRoom && hue < 180 - searchingRoom) {
            lower1 = new Scalar(hue - searchingRoom, 70, 50);
            upper1 = new Scalar(hue + searchingRoom, 255, 255);
        } else if (hue <= searchingRoom) {
            lower1 = new Scalar(1, 70, 50);
            upper1 = new Scalar(hue + searchingRoom, 255, 255);
            lower2 = new Scalar(180 - (hue - searchingRoom), 70, 50);
            upper2 = new Scalar(180, 255, 255);
        } else {
            lower1 = new Scalar(hue - searchingRoom, 70, 50);
            upper1 = new Scalar(180, 255, 255);
            lower2 = new Scalar(1, 70, 50);
            upper2 = new Scalar((hue + searchingRoom) % 180, 255, 255);
        }

        Core.inRange(hsv, lower1, upper1, mask);
        if (lower2 != null && upper2 != null) {
            Mat mask2 = new Mat();
            Core.inRange(hsv, lower2, upper2, mask2);
            Core.bitwise_or(mask, mask2, mask);
        }

        return mask;
    }

    private Mat findLines(Mat color, int ksize, int thresh1, int thresh2) {
        Mat con = new Mat();
        Mat lines = new Mat();
        Imgproc.medianBlur(color, con, ksize);
        Imgproc.Canny(con, lines, thresh1, thresh2);
        return lines;
    }
}
