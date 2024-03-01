package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Autonomous
public class TestEasyOpenCVDrivetrain extends LinearOpMode{
    public DcMotor lf, lb, rf, rb;

    HardwareMap hwMap;

    public void init(HardwareMap map) {


        //Assigns the parent hardware map to local HardwareMap class variable
        hwMap = map;


        //Hardware initialized and String Names are in the Configuration File for Hardware Map


        // Control HUb
        lf = hwMap.get(DcMotor.class, "lf");
        lb = hwMap.get(DcMotor.class, "lb");
        rf = hwMap.get(DcMotor.class, "rf");
        rb = hwMap.get(DcMotor.class, "rb");


        //Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards

        //THIS IS THE CORRECT ORIENTATION
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        ///Reverses shooter motor to shoot the correct way and same with the conveyor motor


        //We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //The 4 wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

    }

    public void power(double output){
        lf.setPower(-output);
        lb.setPower(-output);
        rf.setPower(output);
        rb.setPower(output);
    }
    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        //Wheel powers calculated using game pad 1's inputs leftStickY, leftStickX, and rightStickX

        double lfPower = leftStickY + leftStickX + rightStickX;
        double lbPower = leftStickY - leftStickX + rightStickX;
        double rfPower = leftStickY - leftStickX - rightStickX;
        double rbPower = leftStickY + leftStickX - rightStickX;

        //Sets the wheel's power

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}