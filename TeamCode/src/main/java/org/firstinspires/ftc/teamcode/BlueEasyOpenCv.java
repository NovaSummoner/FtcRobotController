package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueEasyOpenCv extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;


    static final Rect RIGHT_ROI = new Rect(
            new Point(60,35),
            new Point(120,75)
    );
    static final Rect LEFT_ROI = new Rect(
            new Point(140,35),
            new Point(200,75)
    );
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public BlueEasyOpenCv(Telemetry t) { telemetry = t; }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(26, 35, 255);
        Scalar highHSV = new Scalar(0, 141, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);


        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean coneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean coneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (coneLeft && coneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Cone Location", "not found");
        }
        else if (coneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Cone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Cone Location", "left");
        }

        telemetry.update();


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);


        Scalar coneDetector1  = new Scalar(59, 49, 235);
        Scalar coneDetector2 = new Scalar(59, 49, 235);


        Imgproc.rectangle(mat, LEFT_ROI, location == location.LEFT? coneDetector2:coneDetector1);
        Imgproc.rectangle(mat, RIGHT_ROI, location == location.RIGHT? coneDetector2:coneDetector1);


        return mat;
    }


    public Location getLocation() {return location;}
}
