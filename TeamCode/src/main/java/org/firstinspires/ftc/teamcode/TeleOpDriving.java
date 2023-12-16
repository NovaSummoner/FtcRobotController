package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpDriving", group = "TeleOp")
public class TeleOpDriving extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor hangMotor;
    DcMotor armMotor;
    DcMotor laArmMotor;


    Servo prepClaw;
    CRServo claw;
    CRServo airLauncher;
    Servo pixelP;
    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        hangMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        laArmMotor = hardwareMap.dcMotor.get("laArmMotor");
        prepClaw = hardwareMap.servo.get("prepClaw");
        claw = hardwareMap.crservo.get("claw");
        airLauncher = hardwareMap.crservo.get("airLauncher");
        pixelP = hardwareMap.servo.get("pixelP");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        laArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

            //Arm motor
            if (Math.abs(gamepad2.left_stick_y) > .2) {
                armMotor.setPower(1);
            } else {
                armMotor.setPower(0);
            }
            if (Math.abs(-gamepad2.left_stick_y) > .2) {
                armMotor.setPower(-1);
            } else {
                armMotor.setPower(0);
            }

            //Drone launcher
            //if (gamepad2.right_bumper) {
               //airLauncher.setPower(1);
            //} else {
                //airLauncher.setPower(0);
            //}*/

            //Claw servos
            if (gamepad2.dpad_down) {
                airLauncher.setPower(1);
            } else {
                airLauncher.setPower(0);
            }

            if (gamepad2.dpad_right) {
                claw.setPower(1);
                telemetry.addData("Claw Power", claw.getPower());
            } else {
                claw.setPower(0);
            }

            //pixel dropper
            if (gamepad2.a) {
                pixelP.setPosition(1);
                telemetry.addData("PixelP", pixelP.getPosition());
            } else {
                pixelP.setPosition(0);
            }
        }
    }
}



