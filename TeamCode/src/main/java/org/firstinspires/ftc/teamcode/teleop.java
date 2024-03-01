package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class teleop extends OpMode {
    DcMotor lf, lb, rf, rb, intakeMotor, hangingMotor;
    CRServo drone;
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        hangingMotor = hardwareMap.dcMotor.get("hanging");
        drone = hardwareMap.crservo.get("drone");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            rf.setPower(-gamepad1.right_stick_y * 1);
            rb.setPower(-gamepad1.right_stick_y * 1);
        } else {
            rb.setPower(0);
            rf.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_y) > .2) {
            lf.setPower(gamepad1.left_stick_y * 1);
            lb.setPower(-gamepad1.left_stick_y * 1);
        } else {
            lf.setPower(0);
            lb.setPower(0);
        }
        if (gamepad2.a) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad2.b) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad2.x) {
            drone.setPower(1);
        } else {
            drone.setPower(0);
        }

        if (gamepad2.y) {
            drone.setPower(-1);
        } else {
            drone.setPower(0);
        }

        if (gamepad2.dpad_up) {
            hangingMotor.setPower(1);
        } else {
            hangingMotor.setPower(0);
        }

        if (gamepad2.dpad_down) {
            hangingMotor.setPower(-1);
        } else {
            hangingMotor.setPower(0);
        }

    }
}
