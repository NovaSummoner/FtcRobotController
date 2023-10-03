package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous
public class MyOdometryOpmode extends LinearOpMode {
    DcMotor Right_Front, Left_Front, Right_Back, Left_Back;
    DcMotor Tracking_Right, Tracking_Left, Tracking_Middle;
    BNO055IMU imu;
    String Right_FrontName = "rf", Left_FrontName = "lf", Right_BackName = "rb", Left_BackName = "lb";
    String Tracking_RightEncoderName = Right_FrontName, Tracking_LeftEncoderName = Left_FrontName, Tracking_MiddleEncoderName = Right_BackName;
    final double PIVOT_SPEED = 0.8;
    final double COUNTS_PER_INCH = 537.7;
    ElapsedTime timer = new ElapsedTime();
    double Tracking_MiddleTickOffset = 0;
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File Tracking_MiddleTickOffsetFile = AppUtil.getInstance().getSettingsFile("Tracking_MiddleTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap(Right_FrontName, Left_FrontName, Right_BackName, Left_BackName, Tracking_RightEncoderName, Tracking_LeftEncoderName, Tracking_MiddleEncoderName);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        while (getZAngle() < 90 && opModeIsActive()) {
            Right_Front.setPower(-PIVOT_SPEED);
            Right_Back.setPower(-PIVOT_SPEED);
            Left_Front.setPower(PIVOT_SPEED);
            Left_Back.setPower(-PIVOT_SPEED);
            if (getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED);
            } else {
                setPowerAll(-PIVOT_SPEED / 0/5, -PIVOT_SPEED / 0.5, PIVOT_SPEED / 0.5, -PIVOT_SPEED / 0.5);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();

            setPowerAll(0, 0, 0, 0);
            timer.reset();
            while (timer.milliseconds() < 5000 && opModeIsActive()) {
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
            double angle = getZAngle();
            double encoderDifference = Math.abs(Tracking_Left.getCurrentPosition()) + (Math.abs(Tracking_Right.getCurrentPosition()));
            double Middle_TrackingEncoderTickOffsetPerDegree = encoderDifference / angle;
            double wheelBaseSeparation = (2 * 90 * Middle_TrackingEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);
            Tracking_MiddleTickOffset = Tracking_Middle.getCurrentPosition() / Math.toRadians(getZAngle());

            ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
            ReadWriteFile.writeFile(Tracking_MiddleTickOffsetFile, String.valueOf(Tracking_MiddleTickOffset));

            while (opModeIsActive()) {
                telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
                telemetry.addData("Wheel Base Seperation", wheelBaseSeparation);
                telemetry.addData("Tracking_Middle Encoder Offset", Tracking_MiddleTickOffset);

                telemetry.addData("IMU Angle", getZAngle());
                telemetry.addData("Tracking Left Position", Tracking_Left.getCurrentPosition());
                telemetry.addData("Tracking Right Position", Tracking_Right.getCurrentPosition());
                telemetry.addData("Tracking Middle Position", Tracking_Middle.getCurrentPosition());

                telemetry.update();

            }
        }

    }

    private void initHardwareMap(String rfName, String lfName, String rbName, String lbName, String trackingRightEncoderName, String trackingLeftEncoderName, String trackingMiddleEncoderName) {
        Right_Front = hardwareMap.dcMotor.get(rfName);
        Right_Back = hardwareMap.dcMotor.get(rbName);
        Left_Front = hardwareMap.dcMotor.get(lfName);
        Left_Back = hardwareMap.dcMotor.get(lbName);

        Tracking_Left = hardwareMap.dcMotor.get(trackingLeftEncoderName);
        Tracking_Right = hardwareMap.dcMotor.get(trackingRightEncoderName);
        Tracking_Middle = hardwareMap.dcMotor.get(trackingMiddleEncoderName);

        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Tracking_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Tracking_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Tracking_Middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Tracking_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Tracking_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Tracking_Middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private double getZAngle() {return (-imu.getAngularOrientation().firstAngle);}

    private void setPowerAll(double rf, double rb, double lf, double lb) {
        Right_Front.setPower(rf);
        Right_Back.setPower(rb);
        Left_Front.setPower(lf);
        Left_Back.setPower(lb);
    }
}


