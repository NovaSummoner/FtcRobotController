package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTest extends LinearOpMode {
    Servo inOutPrepServo;

    @Override
    public void runOpMode() throws InterruptedException {
        inOutPrepServo = hardwareMap.servo.get("Prep");
        inOutPrepServo.setPosition(100);
        waitForStart();
        inOutPrepServo.setPosition(0);
        sleep(10000);
    }
}
