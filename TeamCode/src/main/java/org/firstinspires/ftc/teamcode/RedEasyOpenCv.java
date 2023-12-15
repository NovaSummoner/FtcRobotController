package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.PIDF_Arm.d;
import static org.firstinspires.ftc.teamcode.PIDF_Arm.f;
import static org.firstinspires.ftc.teamcode.PIDF_Arm.i;
import static org.firstinspires.ftc.teamcode.PIDF_Arm.p;
import static org.firstinspires.ftc.teamcode.PIDF_Arm.target;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Config
@TeleOp
public class RedEasyOpenCv extends LinearOpMode {
    DcMotor right_front, right_back, left_front, left_back;
    BNO055IMU imu;
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.77953;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double WEBCAM_WIDTH = 640;
    private OpenCvCamera webcam;
    private RedContourPipeline pipeline;
    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3;
    private double rightBarcodeRangeBoundary = 0.7;

    private ElapsedTime runtime = new ElapsedTime();

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 200.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 128.0);
    private PIDController controller;
    private DcMotorEx arm_motor;
    private final double ticks_in_degrees = 2786.2 / 180.0;
    private static int armTarget = 0;
    private long encoderDriveStartTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        pipeline = new RedContourPipeline(0.0, 0.0, 0.0, 0.0);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        left_front = hardwareMap.dcMotor.get("lf");
        right_front = hardwareMap.dcMotor.get("rf");
        left_back = hardwareMap.dcMotor.get("lb");
        right_back = hardwareMap.dcMotor.get("rb");

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d : %7d: %7d ",
                left_front.getCurrentPosition(),
                right_front.getCurrentPosition(),
                left_back.getCurrentPosition(),
                right_back.getCurrentPosition());
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

            int armPos = arm_motor.getCurrentPosition();
            double pid = controller.calculate(armPos, armTarget);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;
            arm_motor.setPower(power);

            if(rectangleArea > minRectangleArea){

                if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * WEBCAM_WIDTH){
                    encoderDriveStartTime = System.currentTimeMillis() + 2000;
                    telemetry.addData("Barcode Position", "Right");
                    if (encoderDriveStartTime > 0 && System.currentTimeMillis() >= encoderDriveStartTime) {
                        encoderDrive(0.2,10,10,3);
                        }
                    encoderDriveStartTime = 0;
                    }
                }
                else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * WEBCAM_WIDTH){
                encoderDriveStartTime = System.currentTimeMillis() + 2000;
                telemetry.addData("Barcode Position", "Left");
                    if (encoderDriveStartTime > 0 && System.currentTimeMillis() >= encoderDriveStartTime) {
                    encoderDrive(0.2,10,10,3);
                    }
                encoderDriveStartTime = 0;

            }
                else {
                encoderDriveStartTime = System.currentTimeMillis() + 2000;
                telemetry.addData("Barcode Position", "Center");
                    if (encoderDriveStartTime > 0 && System.currentTimeMillis() >= encoderDriveStartTime) {
                    encoderDrive(0.2,10,10,3);
                    }
                encoderDriveStartTime = 0;
                }
            }
        telemetry.update();
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
            left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = left_front.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = left_back.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = right_front.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = right_back.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            left_front.setTargetPosition(newFrontLeftTarget);
            left_back.setTargetPosition(newBackLeftTarget);
            right_front.setTargetPosition(newFrontRightTarget);
            right_back.setTargetPosition(newBackRightTarget);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            left_front.setPower(Math.abs(speed));
            left_back.setPower(Math.abs(speed));
            right_front.setPower(Math.abs(speed));
            right_back.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_front.isBusy() && right_front.isBusy() && left_back.isBusy() && right_back.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        left_front.getCurrentPosition(),
                        left_back.getCurrentPosition(),
                        right_front.getCurrentPosition(),
                        right_back.getCurrentPosition());
                telemetry.update();
            }
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
    public void encoderDriveStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = left_front.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = left_back.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = right_front.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = right_back.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            left_front.setTargetPosition(newFrontLeftTarget);
            left_back.setTargetPosition(newBackLeftTarget);
            right_front.setTargetPosition(newFrontRightTarget);
            right_back.setTargetPosition(newBackRightTarget);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            left_front.setPower(Math.abs(-speed));
            left_back.setPower(Math.abs(speed));
            right_front.setPower(Math.abs(speed));
            right_back.setPower(Math.abs(-speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_front.isBusy() && right_front.isBusy() && left_back.isBusy() && right_back.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        left_front.getCurrentPosition(),
                        left_back.getCurrentPosition(),
                        right_front.getCurrentPosition(),
                        right_back.getCurrentPosition());
                telemetry.update();
            }

            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);

            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
    private double getZAngle() {return (-imu.getAngularOrientation().firstAngle);
    }

}