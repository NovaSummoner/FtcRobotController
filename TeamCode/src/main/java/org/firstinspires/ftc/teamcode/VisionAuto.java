package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class VisionAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Camer1"))
                .setCameraResolution(new Size(2304, 1536))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                Telemetry dashboardTelemetry = dashboard.getTelemetry();

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

        }
    }
}
