
package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "test: AprilTag", group = "Concept")
public class AprilTagTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Camera VID/PID for the optional USB compatibility quirk
    private static final int VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY = 0x1BCF;
    private static final int PRODUCT_ID_ARDUCAM_OV5648 = 0x284C;

    @Override
    public void runOpMode() {

        // Send telemetry to both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // Optional camera compatibility quirk
        CameraCompatibilityManager.getInstance().addQuirk(
                VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY,
                PRODUCT_ID_ARDUCAM_OV5648,
                CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE
        );

        // Initialize AprilTag processor AND the single VisionPortal
        initAprilTag();

        // Stream the same VisionPortal to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        telemetry.addLine("Camera initialized.");
        telemetry.addLine("Open FTC Dashboard to view the webcam.");
        telemetry.addLine("Press START to begin.");
        telemetry.addLine("D-pad UP: Resume camera");
        telemetry.addLine("D-pad DOWN: Stop camera stream");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Report detected AprilTags
            telemetryAprilTag();
            telemetry.update();

            // Control camera streaming with the gamepad
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            sleep(20);
        }

        // Stop Dashboard stream and release the camera
        FtcDashboard.getInstance().stopCameraStream();

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize the AprilTag processor and VisionPortal.
     */
    private void initAprilTag() {

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create one VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Select camera
        if (USE_WEBCAM) {
            builder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam 1")
            );
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Optional camera resolution
        // Uncomment if your webcam supports this resolution:
        // builder.setCameraResolution(new Size(640, 480));

        // Enable AprilTag detection on this portal
        builder.addProcessor(aprilTag);

        // Build and save the portal in the class field
        visionPortal = builder.build();
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections =
                aprilTag.getDetections();

        telemetry.addData(
                "# AprilTags Detected",
                currentDetections.size()
        );

        for (AprilTagDetection detection : currentDetections) {

            if (detection instanceof AprilTagSingleDetection) {

                AprilTagSingleDetection singleDet =
                        (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {

                    telemetry.addLine(String.format(
                            "\n==== (ID %d) %s",
                            singleDet.id,
                            singleDet.metadata.name
                    ));

                    telemetry.addLine(String.format(
                            "XYZ %6.1f %6.1f %6.1f (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z
                    ));

                    telemetry.addLine(String.format(
                            "PRY %6.1f %6.1f %6.1f (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw
                    ));

                    telemetry.addLine(String.format(
                            "RBE %6.1f %6.1f %6.1f (inch, deg, deg)",
                            detection.ftcPose.range,
                            detection.ftcPose.bearing,
                            detection.ftcPose.elevation
                    ));

                } else {

                    telemetry.addLine(String.format(
                            "\n==== (ID %d) Unknown",
                            singleDet.id
                    ));

                    telemetry.addLine(String.format(
                            "Center %6.0f %6.0f (pixels)",
                            singleDet.center.x,
                            singleDet.center.y
                    ));
                }

            } else if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet =
                        (AprilTagClusterDetection) detection;

                telemetry.addLine(String.format(
                        "\n==== Tag Cluster (%s)",
                        clusterDet.metadata != null
                                ? clusterDet.metadata.name
                                : "Unknown"
                ));

                telemetry.addLine(String.format(
                        "Percent tags found: %d",
                        clusterDet.percentClusterFound
                ));

                telemetry.addLine(String.format(
                        "XYZ %6.1f %6.1f %6.1f (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                ));

                telemetry.addLine(String.format(
                        "PRY %6.1f %6.1f %6.1f (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw
                ));

                telemetry.addLine(String.format(
                        "RBE %6.1f %6.1f %6.1f (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                ));
            }
        }

        telemetry.addLine("\nKey:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up)");
        telemetry.addLine("PRY = Pitch, Roll, Yaw");
        telemetry.addLine("RBE = Range, Bearing, Elevation");
    }
}
