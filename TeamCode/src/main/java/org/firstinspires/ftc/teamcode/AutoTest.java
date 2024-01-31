package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoTest extends LinearOpMode {
    DcMotor right_front, right_back, left_front, left_back;
    Servo pixelP;
    Servo backboardPlacer;
    @Override
    public void runOpMode() throws InterruptedException {
        left_front = hardwareMap.dcMotor.get("lf");
        right_front = hardwareMap.dcMotor.get("rf");
        left_back = hardwareMap.dcMotor.get("lb");
        right_back = hardwareMap.dcMotor.get("rb");
        pixelP = hardwareMap.servo.get("pixelPlacer");
        backboardPlacer = hardwareMap.servo.get("bPlacer");

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        backboardPlacer.setPosition(0.35);
        sleep(30000);
    }
}
