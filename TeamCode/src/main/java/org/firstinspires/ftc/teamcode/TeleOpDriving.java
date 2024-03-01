package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpDriving", group = "TeleOp")
public class TeleOpDriving extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor intake;
    Servo pixelP;
    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        intake = hardwareMap.dcMotor.get("intake");
        pixelP = hardwareMap.servo.get("pixelPlacer");
        /*lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    @Override
    public void loop() {
        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            rf.setPower(-gamepad1.right_stick_y * 1);
            rb.setPower(-gamepad1.right_stick_y * 1);
        } else {
            rb.setPower(0);
            rf.setPower(0);
        }
        //Front back Left
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            lf.setPower(-gamepad1.left_stick_y * 1);
            lb.setPower(-gamepad1.left_stick_y * 1);
        } else {
            lf.setPower(0);
            lb.setPower(0);
        }
        if (Math.abs(-gamepad1.right_trigger) > .2) {
            lf.setPower(0.8);
            rb.setPower(-0.8);
            lb.setPower(-0.8);
            rf.setPower(0.8);
        } else {
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
        }
        if (Math.abs(gamepad1.left_trigger) > .2) {
            lf.setPower(-0.8);
            rb.setPower(0.8);
            lb.setPower(0.8);
            rf.setPower(-0.8);
        } else {
            rf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);
            lf.setPower(0);
        }
        if (gamepad1.a) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if (gamepad1.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }
}



