package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EasyOpenCv extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect RIGHT_ROI = new Rect(
            new Point(),
            new Point()
    );
    public EasyOpenCv(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(217, 92, 38);
        Scalar highHSV = new Scalar(217, 56, 74);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);

        double rightValue = Core.sumElems(right).val[0] /
    }
}
