package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedBackboardEasyOpenCv extends LinearOpMode {
    DcMotor lf, lb, rf, rb;
    Servo claw;
    BNO055IMU imu;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;

    static final double WEBCAM_WIDTH = 640;
    private OpenCvCamera webcam;
    private RedContourPipeline pipeline;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3;
    private double rightBarcodeRangeBoundary = 0.7;
    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 200.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 128.0);


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        pipeline = new RedContourPipeline(0.0, 0.0, 0.0, 0.0);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d : %7d: %7d ",
                lf.getCurrentPosition(),
                rf.getCurrentPosition(),
                lb.getCurrentPosition(),
                rb.getCurrentPosition());
        telemetry.update();

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive())
        {
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            double rectangleArea = pipeline.getRectArea();

            telemetry.addData("Rectangle Area", rectangleArea);
            telemetry.addData("XY: ",  pipeline.getRectMidpointXY());

            if(rectangleArea > minRectangleArea){

                if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * WEBCAM_WIDTH){
                    telemetry.addData("Barcode Position", "Right");
                    encoderDrive(1, 10, 10,5.0);
                    encoderDrive(1, 10, 10, 5.0);
                    encoderDrive(1,10,10,5.0);
                    claw.setPosition (.8);
                    sleep(1000);
                }
                else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * WEBCAM_WIDTH){
                    telemetry.addData("Barcode Position", "Left");
                    encoderDrive(1, 10, 10, 5.0);
                    encoderDrive(1, 10,10,5.0);
                }
                else {
                    telemetry.addData("Barcode Position", "Center");
                }
            }
            telemetry.update();
        }
    }

    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = lf.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = lb.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rb.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            lf.setTargetPosition(newFrontLeftTarget);
            lb.setTargetPosition(newBackLeftTarget);
            rf.setTargetPosition(newFrontRightTarget);
            rb.setTargetPosition(newBackRightTarget);

            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            lf.setPower(Math.abs(speed));
            lb.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            rb.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        lf.getCurrentPosition(),
                        lb.getCurrentPosition(),
                        rf.getCurrentPosition(),
                        rb.getCurrentPosition());
                telemetry.update();
            }
            lf.setPower(0);
            lb.setPower(0);
            rf.setPower(0);
            rb.setPower(0);

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
    public void encoderDriveStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = lf.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = lb.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rb.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newFrontLeftTarget);
            lb.setTargetPosition(newBackLeftTarget);
            rf.setTargetPosition(newFrontRightTarget);
            rb.setTargetPosition(newBackRightTarget);

            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            lf.setPower(Math.abs(-speed));
            lb.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            rb.setPower(Math.abs(-speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        lf.getCurrentPosition(),
                        lb.getCurrentPosition(),
                        rf.getCurrentPosition(),
                        rb.getCurrentPosition());
                telemetry.update();
            }

            lf.setPower(0);
            lb.setPower(0);
            rf.setPower(0);
            rb.setPower(0);

            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
}
