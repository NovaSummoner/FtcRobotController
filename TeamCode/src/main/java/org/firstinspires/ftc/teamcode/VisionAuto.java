package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;

@Autonomous
public class VisionAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCvCamera camera;

        int  cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(2304, 1536, SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Camera1"))
                .setCameraResolution(new Size(2304, 1536))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {


            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);


                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

                dashboardTelemetry.addData("x", tag.ftcPose.x);
                dashboardTelemetry.addData("y", tag.ftcPose.y);
                dashboardTelemetry.addData("z", tag.ftcPose.z);
                dashboardTelemetry.addData("roll", tag.ftcPose.roll);
                dashboardTelemetry.addData("pitch", tag.ftcPose.pitch);
                dashboardTelemetry.addData("yaw", tag.ftcPose.yaw);

                dashboardTelemetry.update();

            }

            telemetry.update();

            while (!isStarted()) {
                telemetry.addData("ROTATION: ", tagProcessor.getDetections());
                telemetry.update();
                FtcDashboard.getInstance().startCameraStream(camera, 60);

                dashboardTelemetry.addData("ROTATION: ", tagProcessor.getDetections());
                telemetry.update();
            }
        }
    }
}

//hamburger