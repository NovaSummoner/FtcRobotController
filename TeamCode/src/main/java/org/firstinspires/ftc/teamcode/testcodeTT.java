package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "testcodeTT")
public class testcodeTT extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FR;
        DcMotor BR;
        DcMotor FL;
        DcMotor BL;
        {
            FR=hardwareMap.dcMotor.get("FR");
            BR=hardwareMap.dcMotor.get("BR");
            FL=hardwareMap.dcMotor.get("FL");
            BL=hardwareMap.dcMotor.get("BL");
            waitForStart();
            FR.setPower(.25);
            BR.setPower(.25);
            FL.setPower(-.25);
            BL.setPower(-.25);
            sleep(5000);
        }
    }
}
