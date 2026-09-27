package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.Bitmap;
import android.util.Base64;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.localizationConstants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;

public class Vision {

    public static class CameraConfig {
        public final double offsetX;        
        public final double offsetY;        
        public final double offsetHeading;  

        public CameraConfig(double offsetX, double offsetY, double offsetHeading) {
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.offsetHeading = offsetHeading;
        }
    }

    private final HardwareMap hardwareMap;
    
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    private AprilTagProcessor aprilTagProcessor1;
    private AprilTagProcessor aprilTagProcessor2;
    
    private final FrameEncoderProcessor frameEncoder1;
    private final FrameEncoderProcessor frameEncoder2;

    private volatile double camX = 0.0;
    private volatile double camY = 0.0;
    private volatile double camHeading = 0.0;

    private volatile int latestTagId = 0;
    private volatile double latestRawX = 0.0;
    private volatile double latestRawY = 0.0;
    private volatile double latestRawZ = 0.0;
    
    private final Object detectionsLock = new Object();
    private final List<AprilTagDetection> activeDetections = new ArrayList<>();

    private final CameraConfig camera1Config = new CameraConfig(-1.0 * 0.0254, -7.5 * 0.0254, -Math.PI / 2.0); // Webcam 1: Right Side
    private final CameraConfig camera2Config = new CameraConfig(-1.0 * 0.0254,  7.5 * 0.0254,  Math.PI / 2.0); // Webcam 2: Left Side

    public Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.frameEncoder1 = new FrameEncoderProcessor();
        this.frameEncoder2 = new FrameEncoderProcessor();
        initVision();
    }

    private void initVision() {
        try {
            AprilTagMetadata customTag1 = new AprilTagMetadata(1, "Tag 1", 0.16986, DistanceUnit.METER);
            AprilTagMetadata customTag2 = new AprilTagMetadata(2, "Tag 2", 0.16986, DistanceUnit.METER);
            AprilTagMetadata customTag3 = new AprilTagMetadata(3, "Tag 3", 0.16986, DistanceUnit.METER);
            AprilTagMetadata customTag4 = new AprilTagMetadata(4, "Tag 4", 0.16986, DistanceUnit.METER);
            
            AprilTagLibrary customLibrary = new AprilTagLibrary.Builder()
                .addTag(customTag1).addTag(customTag2).addTag(customTag3).addTag(customTag4).build();

            aprilTagProcessor1 = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(customLibrary)
                    .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                    .build();

            aprilTagProcessor2 = new AprilTagProcessor.Builder()
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(customLibrary)
                    .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                    .build();

            WebcamName webcam1Name = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
            if (webcam1Name != null) {
                visionPortal1 = new VisionPortal.Builder()
                        .setCamera(webcam1Name)
                        .setCameraResolution(new android.util.Size(320, 240))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .addProcessor(aprilTagProcessor1)
                        .addProcessor(frameEncoder1)
                        .build();
            }

            WebcamName webcam2Name = hardwareMap.tryGet(WebcamName.class, "Webcam 2");
            if (webcam2Name != null) {
                visionPortal2 = new VisionPortal.Builder()
                        .setCamera(webcam2Name)
                        .setCameraResolution(new android.util.Size(320, 240))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .addProcessor(aprilTagProcessor2)
                        .addProcessor(frameEncoder2)
                        .build();
            }
                
        } catch (Exception e) {}
    }

    public void update() {
        List<AprilTagDetection> detections1 = (aprilTagProcessor1 != null) ? aprilTagProcessor1.getDetections() : null;
        List<AprilTagDetection> detections2 = (aprilTagProcessor2 != null) ? aprilTagProcessor2.getDetections() : null;

        synchronized (detectionsLock) {
            activeDetections.clear();
            if (detections1 != null) activeDetections.addAll(detections1);
            if (detections2 != null) activeDetections.addAll(detections2);
        }

        AprilTagDetection bestDetection = null;
        CameraConfig bestConfig = null;
        double closestDistance = Double.MAX_VALUE;

        if (detections1 != null) {
            for (AprilTagDetection detection : detections1) {
                if (detection.ftcPose != null && detection.ftcPose.range < closestDistance) {
                    closestDistance = detection.ftcPose.range;
                    bestDetection = detection;
                    bestConfig = camera1Config;
                }
            }
        }

        if (detections2 != null) {
            for (AprilTagDetection detection : detections2) {
                if (detection.ftcPose != null && detection.ftcPose.range < closestDistance) {
                    closestDistance = detection.ftcPose.range;
                    bestDetection = detection;
                    bestConfig = camera2Config;
                }
            }
        }

        if (bestDetection != null) {
            this.latestTagId = bestDetection.id;
            this.latestRawX = detectionPoseX(bestDetection);
            this.latestRawY = detectionPoseY(bestDetection);
            this.latestRawZ = detectionPoseZ(bestDetection);

            computePose(bestDetection, bestConfig);
        } else {
            this.latestTagId = 0; 
        }
    }

    private double detectionPoseX(AprilTagDetection d) { return d.ftcPose != null ? d.ftcPose.x : 0.0; }
    private double detectionPoseY(AprilTagDetection d) { return d.ftcPose != null ? d.ftcPose.y : 0.0; }
    private double detectionPoseZ(AprilTagDetection d) { return d.ftcPose != null ? d.ftcPose.z : 0.0; }

    private void computePose(AprilTagDetection detection, CameraConfig cam) {
        double tagFieldX = 0.0;
        double tagFieldY = 0.0;
        double tagOrientation = 0.0;

        switch (detection.id) {
            case 1: 
                tagFieldX = 3.66; tagFieldY = 1.83; tagOrientation = Math.PI;
                break;
            case 2: 
                tagFieldX = 1.83; tagFieldY = 0.0; tagOrientation = Math.PI / 2.0;
                break;
            case 3: 
                tagFieldX = 0.0;  tagFieldY = 1.83; tagOrientation = 0.0;
                break;
            case 4: 
                tagFieldX = 1.83; tagFieldY = 3.66; tagOrientation = -Math.PI / 2.0;
                break;
            default: return; 
        }

        double rangeMeters = detection.ftcPose.range; 
        double bearingRad = detection.ftcPose.bearing;
        double yawRad = detection.ftcPose.yaw;

        double alpha = tagOrientation + Math.PI - yawRad;

        double globalRobotHeading = AngleUnit.RADIANS.normalize(alpha - bearingRad - cam.offsetHeading);
        this.camHeading = globalRobotHeading;

        double globalCamX = tagFieldX - (rangeMeters * Math.cos(alpha));
        double globalCamY = tagFieldY - (rangeMeters * Math.sin(alpha));

        double fieldOffsetX = (cam.offsetX * Math.cos(globalRobotHeading)) - (cam.offsetY * Math.sin(globalRobotHeading));
        double fieldOffsetY = (cam.offsetX * Math.sin(globalRobotHeading)) + (cam.offsetY * Math.cos(globalRobotHeading));

        this.camX = globalCamX - fieldOffsetX;
        this.camY = globalCamY - fieldOffsetY;
    }

    public double getCamX() { return this.camX; }
    public double getCamY() { return this.camY; }
    public double getCamHeading() { return this.camHeading; }

    public int getLatestTagId() { return this.latestTagId; }
    public double getLatestRawX() { return this.latestRawX; }
    public double getLatestRawY() { return this.latestRawY; }
    public double getLatestRawZ() { return this.latestRawZ; }
    
    public List<AprilTagDetection> getCamDetections() {
        synchronized (detectionsLock) {
            return new ArrayList<>(this.activeDetections);
        }
    }
    
    public String getLatestBase64Frame1() { return this.frameEncoder1.getLatestBase64Frame(); }
    public String getLatestBase64Frame2() { return this.frameEncoder2.getLatestBase64Frame(); }

    public void close() {
        if (visionPortal1 != null) visionPortal1.close();
        if (visionPortal2 != null) visionPortal2.close();
        frameEncoder1.cleanup();
        frameEncoder2.cleanup();
    }

    private static class FrameEncoderProcessor implements VisionProcessor {
        private Bitmap reusableBitmap = null;
        private final Mat downscaledMat = new Mat();
        private final Mat grayscaleMat = new Mat();
        private final Size targetSize = new Size(160, 120);
        private final ByteArrayOutputStream baos = new ByteArrayOutputStream();
        private volatile String latestBase64Frame = "";
        
        private long lastEncodeTime = 0;

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
            reusableBitmap = Bitmap.createBitmap(160, 120, Bitmap.Config.ARGB_8888);
        }
        
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (frame == null || frame.empty() || reusableBitmap == null) return null;

            long currentTime = System.currentTimeMillis();
            if (currentTime - lastEncodeTime < localizationConstants.CAMERA_ENCODE) {
                return null;
            }
            lastEncodeTime = currentTime;

            try {
                Imgproc.resize(frame, downscaledMat, targetSize);
                Imgproc.cvtColor(downscaledMat, grayscaleMat, Imgproc.COLOR_RGBA2GRAY);
                Utils.matToBitmap(grayscaleMat, reusableBitmap);
                
                baos.reset();
                reusableBitmap.compress(Bitmap.CompressFormat.JPEG, 25, baos);
                
                latestBase64Frame = Base64.encodeToString(baos.toByteArray(), Base64.NO_WRAP);
            } catch (Exception ignored) {}
            return null;
        }

        public String getLatestBase64Frame() { return this.latestBase64Frame; }

        public void cleanup() {
            downscaledMat.release();
            grayscaleMat.release();
            if (reusableBitmap != null && !reusableBitmap.isRecycled()) {
                reusableBitmap.recycle();
            }
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpToDrawFx, float scaleBmpToDrawFy, Object userContext) {}
    }
}