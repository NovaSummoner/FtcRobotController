package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Meet1TeleOp")
public class Meet1TeleOp extends OpMode {

    DcMotor lf;
    DcMotor rf;
    DcMotor hang;
    DcMotor inTake;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        hang = hardwareMap.dcMotor.get("hang");
        inTake = hardwareMap.dcMotor.get("inTake");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        //Movement
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            rf.setPower(-gamepad1.right_stick_y * 0.8);
        } else {
            rf.setPower(0);
        }
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            lf.setPower(gamepad1.left_stick_y * 0.8);
        } else {
            lf.setPower(0);
        }
        //Intake movement
        if (Math.abs(-gamepad2.left_stick_y) > .2) {
            inTake.setPower(gamepad2.left_stick_y * -0.8);
        } else {
            inTake.setPower(0);
        }
        //Hanging mechanisms
        if (gamepad2.dpad_up) {
            hang.setPower(1);
        } else {
            hang.setPower(0);
        }

        if (gamepad2.dpad_down) {
            hang.setPower(-1);
        } else {
            hang.setPower(0);
        }

    }
}
