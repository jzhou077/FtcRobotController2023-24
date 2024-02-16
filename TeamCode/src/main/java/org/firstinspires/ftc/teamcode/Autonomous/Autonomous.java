package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous")
@TeleOp
public class Autonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "filename.tflite";
    private static final String[] LABELS = {"Pixel"};
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //Lens intrinsics, units are pixels, this calibration is for the C920 webcam at 800x448
    //needs to be updated for different configurations
    private static final double fx = 578.272;
    private static final double fy = 578.272;
    private static final double cx = 402.145;
    private static final double cy = 221.506;
    @Override
    public void runOpMode() throws InterruptedException {

        initProcessors();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

//              addTfodData();
                addAprilTagData();
                telemetry.update();

                sleep(20);
            }
        }
        visionPortal.close();
    }

    private void initProcessors() {
        tfod = new TfodProcessor.Builder().build();
        tfod.setMinResultConfidence(0.5f);

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagOutline(true)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 448))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessors(tfod, aprilTag)
                .build();
    }

    private void addTfodData() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Objects Detected: ", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight())/2;
            double y = (recognition.getTop() + recognition.getBottom())/2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("Position: ", "%.0f / %.0f", x, y);
            telemetry.addData("Size: ", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("Region: ", detectRegion(recognition));
        }
    }

    private int detectRegion(Recognition recognition) {
        double imageWidth = (double) recognition.getImageWidth();
        double center = (recognition.getLeft() + recognition.getRight())/2;

        //1 = left, 2 = center, 3 = right
        if (center < imageWidth/3) {
            return 1;
        }
        else if (center > (imageWidth/3)*2) {
            return 3;
        }
        else {
            return 2;
        }
    }

    private void addAprilTagData() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("April Tags Detected: ", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //ftcpose (robot reference frame) xyz
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //RBE - range, bearing, elevation
                //range - distance to tag
                //bearing - angle to tag
                //yaw - angle of tag
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}
