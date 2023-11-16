package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Disabled
@TeleOp (name = "Concept: Gamepad Touch", group = "Concept")
public class TouchPad extends LinearOpMode
{
    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData(">", "Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean finger = false;
            if (gamepad2.touchpad_finger_1){
                finger = true;
                telemetry.addLine(String.format("Finger 1: x=%5.2f y=%5.2f\n", gamepad2.touchpad_finger_1_x, gamepad2.touchpad_finger_1_y));
            }
            if (gamepad2.touchpad_finger_2){
                finger = true;
                telemetry.addLine(String.format("Finger 1: x=%5.2f y=%5.2f\n", gamepad2.touchpad_finger_2_x, gamepad2.touchpad_finger_2_y));
            }
            if (!finger){
            }
                telemetry.addLine("No fingers");
        }

    }
}

