package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TouchPad;

@TeleOp(name = "TeleOp4motor", group = "TeleOp")
public class fourmotortesting extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor inTakeMotor;
    DcMotor HangingMotor;
    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            lf.setPower(gamepad1.left_stick_y * 1);
            lb.setPower(gamepad1.left_stick_y * 1);
        } else {
            lf.setPower(0);
            lb.setPower(0);


            //Strafe Right
            if (Math.abs(-gamepad1.right_trigger) > .2) {
                lf.setPower(-gamepad1.right_trigger * 1);
                rb.setPower(gamepad1.right_trigger * 1);
                lb.setPower(gamepad1.right_trigger * 1);
                rf.setPower(-gamepad1.right_trigger * 1);

            } else {
                rf.setPower(0);
                lb.setPower(0);
                rb.setPower(0);
                lf.setPower(0);

            }
            //Strafe Left
            if (Math.abs(gamepad1.left_trigger) > .2) {
                lf.setPower(gamepad1.left_trigger * 1);
                rb.setPower(-gamepad1.left_trigger * 1);
                lb.setPower(-gamepad1.left_trigger * 1);
                rf.setPower(gamepad1.left_trigger * 1);

            } else {
                rf.setPower(0);
                rb.setPower(0);
                lb.setPower(0);
                lf.setPower(0);
            }

        }
    }
}



