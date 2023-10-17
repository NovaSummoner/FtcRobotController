package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoDrive")
public class AutoDrive extends LinearOpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;

    @Override
    public void runOpMode() throws InterruptedException {
        {
            lf=hardwareMap.dcMotor.get("lf");
            lb=hardwareMap.dcMotor.get("lb");
            rf=hardwareMap.dcMotor.get("rf");
            rb=hardwareMap.dcMotor.get("rb");
            waitForStart();
            lf.setPower(-0.5);
            lb.setPower(-0.5);
            rf.setPower(0.5);
            rb.setPower(0.5);
            sleep(5000);
        }
    }
}
