package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOpDriving")
public class TeleOpDriving extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor inTakeMotor;
    Servo inTakeServo;
    DcMotor hangingMotor;

    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        inTakeMotor = hardwareMap.dcMotor.get("inTakeMotor");
        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");
        inTakeServo = hardwareMap.servo.get("inTakeServo");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTakeServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {

        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            rf.setPower(-gamepad1.right_stick_y * 0.8);
            rb.setPower(-gamepad1.right_stick_y * 0.8);
        } else {
            rb.setPower(0);
            rf.setPower(0);
        }

        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            lf.setPower(gamepad1.left_stick_y * 0.8);
            lb.setPower(gamepad1.left_stick_y * 0.8);
        } else {
            lf.setPower(0);
            lb.setPower(0);


            //Strafe Right
            if (Math.abs(-gamepad1.right_trigger) > .2) {
                lf.setPower(-gamepad1.right_trigger * 0.8);
                rb.setPower(gamepad1.right_trigger * 0.8);
                lb.setPower(gamepad1.right_trigger * 0.8);
                rf.setPower(-gamepad1.right_trigger * 0.8);

            } else {
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                lf.setPower(0);

            }
            //Strafe Left
            if (Math.abs(gamepad1.left_trigger) > .2) {
                lf.setPower(gamepad1.left_trigger * 0.8);
                rb.setPower(-gamepad1.left_trigger * 0.8);
                lb.setPower(-gamepad1.left_trigger * 0.8);
                rf.setPower(gamepad1.left_trigger * 0.8);

            } else {
                rf.setPower(0);
                rb.setPower(0);
                lb.setPower(0);
                lf.setPower(0);
            }

            //Intake Controller & Motor >:D

            if (Math.abs(-gamepad2.left_stick_y) > .2) {
                inTakeMotor.setPower(gamepad2.left_stick_y * -0.8);
            } else {
                inTakeMotor.setPower(0);
            }

            if (gamepad2.a) {
                inTakeServo.setPosition(1);
            } else {
                inTakeServo.setPosition(0);
            }

            if (gamepad2.b) {
                inTakeServo.setPosition(1);
            } else {
                inTakeServo.setPosition(0);
            }
            //Hanging Mechanisms

            if (gamepad2.dpad_up) {
                hangingMotor.setPower(1);
            } else {
                hangingMotor.setPower(1);
            }

            if (gamepad2.dpad_down) {
                inTakeMotor.setPower(-1);
            } else {
                hangingMotor.setPower(1);
            }
        }
    }
}//hamburger