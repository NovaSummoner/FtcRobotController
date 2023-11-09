package org.firstinspires.ftc.teamcode;

import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
@Autonomous(name = "BlueOpenCvDetector", group="Auto")
public class BlueEasyOpenCvAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvCamera camera;

        int  cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        BlueEasyOpenCv detector = new BlueEasyOpenCv(telemetry);
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { camera.startStreaming(320,240, SIDEWAYS_LEFT); }

            @Override
            public void onError(int errorCode) {

            }

        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        switch(detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        camera.stopStreaming();
    }
}
