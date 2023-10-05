package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpDriving")
public class TeleOpDriving extends OpMode {


    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;


    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        //Front back Left
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            frontLeft.setPower(-gamepad1.left_stick_y * -1);
            backLeft.setPower(-gamepad1.left_stick_y * -1);
        } else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }

        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            frontRight.setPower(-gamepad1.right_stick_y * -1);
            backRight.setPower(-gamepad1.right_stick_y * -1);
        } else {
            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);

        }
            //Front back Right
        if (Math.abs(-gamepad1.left_stick_x) > .2) {
                frontLeft.setPower(-gamepad1.left_stick_x * -1);
                backRight.setPower(-gamepad1.right_stick_x * -1);
                backLeft.setPower(-gamepad1.left_stick_x * -1);
                frontRight.setPower(-gamepad1.right_stick_x * -1);

            } else {
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);

            }
                //Front back Left
            if (Math.abs(gamepad1.right_stick_x) > -1) {
            frontLeft.setPower(-gamepad1.left_stick_x * -1);
            backRight.setPower(gamepad1.right_stick_x * -1);
            backLeft.setPower(gamepad1.left_stick_x * -1);
            frontRight.setPower(-gamepad1.right_stick_x * -1);

        } else{
                frontRight.setPower(0);
                backLeft.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
    }



        }

    }

