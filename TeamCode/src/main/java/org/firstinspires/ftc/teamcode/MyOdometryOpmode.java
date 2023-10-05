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
    DcMotor VerticalRight, VerticalLeft, Horizontal;
    BNO055IMU imu;
    String Right_FrontName = "rf", Left_FrontName = "lf", Right_BackName = "rb", Left_BackName = "lb";
    String VerticalRightEncoderName = Right_BackName, VerticalLeftEncoderName = Left_FrontName, HorizontalEncoderName = Right_FrontName;
    final double PIVOT_SPEED = 5;
    final double COUNTS_PER_INCH = 537.7;
    ElapsedTime timer = new ElapsedTime();
    double HorizontalTickOffset = 0;


    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File HorizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("HorizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap(Right_FrontName, Left_FrontName, Right_BackName, Left_BackName, VerticalRightEncoderName, VerticalLeftEncoderName, HorizontalEncoderName);

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
                setPowerAll(-PIVOT_SPEED / 1, -PIVOT_SPEED / 1, PIVOT_SPEED / 1, -PIVOT_SPEED / 1);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();

            setPowerAll(0, 0, 0, 0);
            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            }
            double angle = getZAngle();
            double encoderDifference = Math.abs(VerticalLeft.getCurrentPosition()) + (Math.abs(VerticalRight.getCurrentPosition()));
            double VerticalEncoderTickOffsetPerDegree = encoderDifference / angle;
            double wheelBaseSeparation = (2 * 90 * VerticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);
            double HorizontalTickOffsetPerDegree = Horizontal.getCurrentPosition() / Math.toRadians(getZAngle());
            HorizontalTickOffset = HorizontalTickOffsetPerDegree;


            ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
            ReadWriteFile.writeFile(HorizontalTickOffsetFile, String.valueOf(HorizontalTickOffset));

            while (opModeIsActive()) {
                telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
                telemetry.addData("Wheel Base Seperation", wheelBaseSeparation);
                telemetry.addData("Horizontal Encoder Offset", HorizontalTickOffset);

                telemetry.addData("IMU Angle", getZAngle());
                telemetry.addData("VerticalLeft Position", VerticalLeft.getCurrentPosition());
                telemetry.addData("VerticalRight Position", VerticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", Horizontal.getCurrentPosition());

                telemetry.update();

            }
        }

    }

    private void initHardwareMap(String rfName, String lfName, String rbName, String lbName, String VerticalRightEncoderName, String VerticalLeftEncoderName, String HorizontalEncoderName) {
        Right_Front = hardwareMap.dcMotor.get(rfName);
        Right_Back = hardwareMap.dcMotor.get(rbName);
        Left_Front = hardwareMap.dcMotor.get(lfName);
        Left_Back = hardwareMap.dcMotor.get(lbName);

        VerticalLeft = hardwareMap.dcMotor.get(VerticalLeftEncoderName);
        VerticalRight = hardwareMap.dcMotor.get(VerticalRightEncoderName);
        Horizontal = hardwareMap.dcMotor.get(HorizontalEncoderName);

        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left_Back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VerticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        VerticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VerticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    private double getZAngle() {return (imu.getAngularOrientation().firstAngle);}

    private void setPowerAll(double rf, double rb, double lf, double lb) {
        Right_Front.setPower(rf);
        Right_Back.setPower(rb);
        Left_Front.setPower(lf);
        Left_Back.setPower(lb);
    }
}


