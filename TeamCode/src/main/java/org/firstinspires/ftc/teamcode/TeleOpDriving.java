package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpDriving")
public class TeleOpDriving extends OpMode {


    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;
    DcMotor inTakeMotor;


    @Override
    public void init() {

        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        inTakeMotor = hardwareMap.dcMotor.get("inTakeMotor");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        //Front back Left
        if (Math.abs(-gamepad1.left_stick_y) > .2) {
            fl.setPower(-gamepad1.left_stick_y * -1);
            bl.setPower(-gamepad1.left_stick_y * -1);
        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }

        //Front back Right
        if (Math.abs(-gamepad1.right_stick_y) > .2) {
            fr.setPower(-gamepad1.right_stick_y * -1);
            br.setPower(-gamepad1.right_stick_y * -1);
        } else {
            fr.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            bl.setPower(0);

        }
        //Front back Right
        if (Math.abs(-gamepad1.left_stick_x) > .2) {
            fl.setPower(-gamepad1.left_stick_x * -1);
            br.setPower(-gamepad1.right_stick_x * -1);
            bl.setPower(-gamepad1.left_stick_x * -1);
            fr.setPower(-gamepad1.right_stick_x * -1);

        } else {
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);

        }
        //Front back Left
        if (Math.abs(gamepad1.right_stick_x) > .2) {
            fl.setPower(-gamepad1.left_stick_x * -1);
            br.setPower(gamepad1.right_stick_x * -1);
            bl.setPower(gamepad1.left_stick_x * -1);
            fr.setPower(-gamepad1.right_stick_x * -1);

        } else {
            fr.setPower(0);
            bl.setPower(0);
            bl.setPower(0);
            fl.setPower(0);
        }

        //Intake Controller & Motor >:D

        if (Math.abs(-gamepad2.left_stick_y) > .2) {
            inTakeMotor.setPower(gamepad2.left_stick_y * -1);
        } else {
            inTakeMotor.setPower(0);

        }

    }
}

