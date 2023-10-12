package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpDriving")
public class TeleOpDriving extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor inTakeMotor;


    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        inTakeMotor = hardwareMap.dcMotor.get("inTakeMotor");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        //Front back Left
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            lf.setPower(gamepad1.left_stick_y * 1);
            lb.setPower(gamepad1.left_stick_y * 1);
        } else {
            lf.setPower(0);
            lb.setPower(0);
            rf.setPower(0);
            rb.setPower(0);
        }

        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            rf.setPower(-gamepad1.right_stick_y * 1);
            rb.setPower(-gamepad1.right_stick_y * 1);
        } else {
            rf.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            lb.setPower(0);

        }
        //Front back Right
        if (Math.abs(-gamepad1.left_stick_x) > .2) {
            lf.setPower(-gamepad1.left_stick_x * 1);
            rb.setPower(-gamepad1.right_stick_x * 1);
            lb.setPower(-gamepad1.left_stick_x * 1);
            rf.setPower(-gamepad1.right_stick_x * 1);

        } else {
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);

        }
        //Front back Left
        if (Math.abs(gamepad1.right_stick_x) > .2) {
            lf.setPower(-gamepad1.left_stick_x * 1);
            rb.setPower(gamepad1.right_stick_x * 1);
            lb.setPower(gamepad1.left_stick_x * 1);
            rf.setPower(-gamepad1.right_stick_x * 1);

        } else {
            rf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);
            lf.setPower(0);
        }

        //Intake Controller & Motor >:D

        if (Math.abs(-gamepad2.left_stick_y) > .2) {
            inTakeMotor.setPower(gamepad2.left_stick_y * -1);
        } else {
            inTakeMotor.setPower(0);

        }

    }
}

//hamburger