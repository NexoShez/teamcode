package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor processor;
    private VisionPortal vPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap map, Telemetry tele, String camName) {
        telemetry = tele;

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        vPortal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, camName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();


    }

    public void update() {
        detectedTags = processor.getDetections();
    }

    public void loop() {

    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagbyID(int id) {
        for (AprilTagDetection det : detectedTags) {
            if (det.id == id) {
//                displayDetTele(det);

                return det;
            }
        }
        return null;
    }

    public boolean isMetadata(AprilTagDetection det) {
        if (det == null) {return false;}

        return det.metadata != null;
    }

    public double getTagRange(AprilTagDetection id) {
        if (id == null) { return 1;}

        if (id.metadata != null) {
            return id.ftcPose.range;
        }

        return 1;
    }

    public void displayDetTele(AprilTagDetection detID){
        if (detID == null) { return; }

        if (detID.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detID.id, detID.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detID.ftcPose.x, detID.ftcPose.y, detID.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detID.ftcPose.pitch, detID.ftcPose.roll, detID.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detID.ftcPose.range, detID.ftcPose.bearing, detID.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detID.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detID.center.x, detID.center.y));
        }

    }

    public void stop() {
        if (vPortal != null) {
            vPortal.close();
        }
    }
}
