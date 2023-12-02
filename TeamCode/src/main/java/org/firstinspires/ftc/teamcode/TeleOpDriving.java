package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TouchPad;

@TeleOp(name = "TeleOpDriving", group = "TeleOp")
public class TeleOpDriving extends OpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    Servo inOutPrepServo;
    CRServo inOutServo;
    CRServo droneServo;
    DcMotor HangingMotor1;
    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        inOutPrepServo = hardwareMap.servo.get("Prep");
        HangingMotor1 = hardwareMap.dcMotor.get("Hang1");
        inOutServo = hardwareMap.crservo.get("inOut");
        droneServo = hardwareMap.crservo.get("drone");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        HangingMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

            //Intake Controller & Motor >:D

            if (gamepad2.a) {
                inOutPrepServo.setPosition(0.5);
            } else if (gamepad2.b) {
                inOutPrepServo.setPosition(100);
            }

            if (gamepad2.x) {
                inOutPrepServo.setPosition(0.78);
            } else if (gamepad2.b) {
                inOutPrepServo.setPosition(100);
            }

            if (Math.abs(-gamepad2.left_stick_y) > 0.2) {
                inOutServo.setPower(gamepad2.left_stick_y * -0.8);
            } else {
                inOutServo.setPower(0);
            }

                if (Math.abs(-gamepad2.right_trigger) > .2) {
                    HangingMotor1.setPower(gamepad2.right_trigger * -0.8);
                } else {
                    HangingMotor1.setPower(0);

                    if (Math.abs(-gamepad2.left_trigger) > .2) {
                        HangingMotor1.setPower(gamepad2.left_trigger * 0.8);
                    } else {
                        HangingMotor1.setPower(0);
                    }
                }
            }

            //drone launcher

        if (gamepad2.dpad_up) {
            droneServo.setPower(-1);
        } else if (gamepad2.dpad_down) {
            droneServo.setPower(0);
        }

        if(gamepad1.touchpad_finger_1) {
                boolean finger = true;
            } else {
                boolean finger = false;
            }
        }
    }



//hamburger