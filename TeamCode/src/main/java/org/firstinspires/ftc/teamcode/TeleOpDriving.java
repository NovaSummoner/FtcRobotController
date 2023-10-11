package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpDriving")
public class TeleOpDriving extends OpMode {


    DcMotor front_left;
    DcMotor back_left;
    DcMotor front_right;
    DcMotor back_right;
    DcMotor inTakeMotor;


    @Override
    public void init() {

        front_left = hardwareMap.dcMotor.get("front_left");
        back_left = hardwareMap.dcMotor.get("back_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_right = hardwareMap.dcMotor.get("back_right");
        inTakeMotor = hardwareMap.dcMotor.get("inTakeMotor");
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        //Front back Left
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            front_left.setPower(-gamepad1.left_stick_y * -1);
            back_left.setPower(-gamepad1.left_stick_y * -1);
        } else {
            front_left.setPower(0);
            back_left.setPower(0);
            front_right.setPower(0);
            back_right.setPower(0);
        }

        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            front_right.setPower(-gamepad1.right_stick_y * -1);
            back_right.setPower(-gamepad1.right_stick_y * -1);
        } else {
            front_right.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);
            back_left.setPower(0);

        }
        //Front back Right
        if (Math.abs(-gamepad1.left_stick_x) > .2) {
            front_left.setPower(-gamepad1.left_stick_x * -1);
            back_right.setPower(-gamepad1.right_stick_x * -1);
            back_left.setPower(-gamepad1.left_stick_x * -1);
            front_right.setPower(-gamepad1.right_stick_x * -1);

        } else {
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);

        }
        //Front back Left
        if (Math.abs(gamepad1.right_stick_x) > .2) {
            front_left.setPower(-gamepad1.left_stick_x * -1);
            back_right.setPower(gamepad1.right_stick_x * -1);
            back_left.setPower(gamepad1.left_stick_x * -1);
            front_right.setPower(-gamepad1.right_stick_x * -1);

        } else {
            front_right.setPower(0);
            back_left.setPower(0);
            back_left.setPower(0);
            front_left.setPower(0);
        }

        //Intake Controller & Motor >:D

        if (Math.abs(-gamepad2.left_stick_y) > .2) {
            inTakeMotor.setPower(gamepad2.left_stick_y * -1);
        } else {
            inTakeMotor.setPower(0);

        }

    }
}

