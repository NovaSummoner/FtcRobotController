package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto", group="Robot")

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor rf = null;
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;


    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 7.55906 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        rf = hardwareMap.get(DcMotor.class, "rf");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lb = hardwareMap.get(DcMotor.class, "lb");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        rf.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                rf.getCurrentPosition(),
                lf.getCurrentPosition(),
                rb.getCurrentPosition(),
                lb.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        encoderStrafe(DRIVE_SPEED, 20, 20, 5.0);  // S2: Strafe Left 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 5, 5, 5.0); // S3: Reverse 10 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newlfTarget;
        int newrfTarget;
        int newlbTarget;
        int newrbTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newlfTarget = lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newrfTarget = rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newlbTarget = lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newrbTarget = rb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newlfTarget);
            rf.setTargetPosition(newrfTarget);
            lb.setTargetPosition(newlbTarget);
            rb.setTargetPosition(newrbTarget);

            // Turn On RUN_TO_POSITION
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rf.setPower(Math.abs(speed));
            lf.setPower(Math.abs(speed));
            rb.setPower(Math.abs(speed));
            lb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rf.isBusy() && rb.isBusy() && lb.isBusy() && lf.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newrfTarget,  newlfTarget, newrbTarget, newlbTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newrfTarget, newlfTarget, newrbTarget, newlbTarget,
                        rf.getCurrentPosition(), lf.getCurrentPosition(), rb.getCurrentPosition(), lb.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rf.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);

            // Turn off RUN_TO_POSITION
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    //strafing encoders
    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS){

        int newflTarget;
        int newfrTarget;
        int newblTarget;
        int newbrTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // For strafing one side's wheels 'attract' each other while the other side 'repels' each other
            //The positive value will make the robot go to the left with current arrangement
            newflTarget = lf.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newfrTarget = rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newblTarget = lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbrTarget = rb.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newflTarget);
            rf.setTargetPosition(newfrTarget);
            lb.setTargetPosition(newblTarget);
            rb.setTargetPosition(newbrTarget);

            // Turn On RUN_TO_POSITION
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rf.setPower(Math.abs(speed));
            lf.setPower(Math.abs(speed));
            rb.setPower(Math.abs(speed));
            lb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rf.isBusy() && rb.isBusy() && lb.isBusy() && lf.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newfrTarget,  newflTarget, newbrTarget, newblTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newfrTarget, newflTarget, newbrTarget, newblTarget,
                        rf.getCurrentPosition(), lf.getCurrentPosition(), rb.getCurrentPosition(), lb.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rf.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);

            // Turn off RUN_TO_POSITION
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //private boolean opModeIsActive() {
    //}
}