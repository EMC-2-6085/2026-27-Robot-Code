package org.firstinspires.ftc.teamcode.SubSystems;

import android.graphics.Bitmap;
import android.util.Base64;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.localizationConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.visionConstants;
import org.firstinspires.ftc.teamcode.Helpers.TagCluster;
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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Vision {

    public enum HiveCluster {
        NONE,
        RED_AUDIENCE,
        RED_BACKSIDE,
        BLUE_AUDIENCE,
        BLUE_BACKSIDE
    }

    private final HardwareMap hardwareMap;
    
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    private AprilTagProcessor aprilTagProcessor1;
    private AprilTagProcessor aprilTagProcessor2;
    
    private final FrameEncoderProcessor frameEncoder1;
    private final FrameEncoderProcessor frameEncoder2;

    private volatile int latestTagId = 0;
    private volatile boolean latestIsUpsideDown = false;
    private volatile HiveCluster activeHiveCluster = HiveCluster.NONE;
    
    private double latestCamX = 0.0;
    private double latestCamY = 0.0;
    private double latestCamHeading = 0.0;
    
    private final Object detectionsLock = new Object();
    private final List<AprilTagDetection> activeDetections = new ArrayList<>();

    public final TagCluster redAudienceCluster = new TagCluster(1);
    public final TagCluster redBacksideCluster = new TagCluster(1);
    public final TagCluster blueAudienceCluster = new TagCluster(-1);
    public final TagCluster blueBacksideCluster = new TagCluster(-1);

    public Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.frameEncoder1 = new FrameEncoderProcessor();
        this.frameEncoder2 = new FrameEncoderProcessor();
        initVision();
    }

    private void initVision() {
        try {
            Set<Integer> uniqueTagIds = new HashSet<>();
            for (int id : visionConstants.RED_AUDIENCE_TAGS) uniqueTagIds.add(id);
            for (int id : visionConstants.RED_BACKSIDE_TAGS) uniqueTagIds.add(id);
            for (int id : visionConstants.BLUE_AUDIENCE_TAGS) uniqueTagIds.add(id);
            for (int id : visionConstants.BLUE_BACKSIDE_TAGS) uniqueTagIds.add(id);

            AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
            for (int id : uniqueTagIds) {
                // 3.25 inches = 0.08255 meters
                libraryBuilder.addTag(new AprilTagMetadata(id, "Tag " + id, 0.08255, DistanceUnit.METER));
            }
            AprilTagLibrary customLibrary = libraryBuilder.build();

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

        redAudienceCluster.clearCluster();
        redBacksideCluster.clearCluster();
        blueAudienceCluster.clearCluster();
        blueBacksideCluster.clearCluster();

        double[][] redAudTags = new double[4][3];
        double[][] redBackTags = new double[4][3];
        double[][] blueAudTags = new double[4][3];
        double[][] blueBackTags = new double[4][3];

        AprilTagDetection bestDetection = null;
        double closestDistance = Double.MAX_VALUE;
        HiveCluster detectedCluster = HiveCluster.NONE;

        for (AprilTagDetection detection : activeDetections) {
            int id = detection.id;

            int idx = getIndexInArray(visionConstants.RED_AUDIENCE_TAGS, id);
            if (idx != -1) detectedCluster = HiveCluster.RED_AUDIENCE;

            idx = getIndexInArray(visionConstants.RED_BACKSIDE_TAGS, id);
            if (idx != -1) detectedCluster = HiveCluster.RED_BACKSIDE;

            idx = getIndexInArray(visionConstants.BLUE_AUDIENCE_TAGS, id);
            if (idx != -1) detectedCluster = HiveCluster.BLUE_AUDIENCE;

            idx = getIndexInArray(visionConstants.BLUE_BACKSIDE_TAGS, id);
            if (idx != -1) detectedCluster = HiveCluster.BLUE_BACKSIDE;

            if (detection.ftcPose == null) continue;

            if (detection.ftcPose.range < closestDistance) {
                closestDistance = detection.ftcPose.range;
                bestDetection = detection;
            }

            double[] poseData = new double[] { detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z };

            idx = getIndexInArray(visionConstants.RED_AUDIENCE_TAGS, id);
            if (idx != -1) redAudTags[idx] = poseData;

            idx = getIndexInArray(visionConstants.RED_BACKSIDE_TAGS, id);
            if (idx != -1) redBackTags[idx] = poseData;

            idx = getIndexInArray(visionConstants.BLUE_AUDIENCE_TAGS, id);
            if (idx != -1) blueAudTags[idx] = poseData;

            idx = getIndexInArray(visionConstants.BLUE_BACKSIDE_TAGS, id);
            if (idx != -1) blueBackTags[idx] = poseData;
        }

        redAudienceCluster.setCluster(redAudTags[0], redAudTags[1], redAudTags[2], redAudTags[3]);
        redBacksideCluster.setCluster(redBackTags[0], redBackTags[1], redBackTags[2], redBackTags[3]);
        blueAudienceCluster.setCluster(blueAudTags[0], blueAudTags[1], blueAudTags[2], blueAudTags[3]);
        blueBacksideCluster.setCluster(blueBackTags[0], blueBackTags[1], blueBackTags[2], blueBackTags[3]);

        this.activeHiveCluster = detectedCluster;

        if (bestDetection != null) {
            this.latestTagId = bestDetection.id;
            this.latestCamX = bestDetection.ftcPose.x;
            this.latestCamY = bestDetection.ftcPose.y;
            this.latestCamHeading = bestDetection.ftcPose.yaw;
            double roll = bestDetection.ftcPose.roll;
            double pitch = bestDetection.ftcPose.pitch;
            this.latestIsUpsideDown = (Math.abs(roll) > Math.PI / 2.0 || Math.abs(pitch) > Math.PI / 2.0);
        } else if (!activeDetections.isEmpty()) {
            this.latestTagId = activeDetections.get(0).id;
            this.latestCamX = 0.0;
            this.latestCamY = 0.0;
            this.latestCamHeading = 0.0;
            this.latestIsUpsideDown = false;
        } else {
            this.latestTagId = 0; 
            this.latestCamX = 0.0;
            this.latestCamY = 0.0;
            this.latestCamHeading = 0.0;
            this.latestIsUpsideDown = false;
        }
    }

    public HiveCluster getActiveCluster() {
        return this.activeHiveCluster;
    }

    public double[] getActiveTargetPosition(Localization localization) {
        switch (activeHiveCluster) {
            case RED_AUDIENCE:
                return redAudienceCluster.getTargetPosition(localization);
            case RED_BACKSIDE:
                return redBacksideCluster.getTargetPosition(localization);
            case BLUE_AUDIENCE:
                return blueAudienceCluster.getTargetPosition(localization);
            case BLUE_BACKSIDE:
                return blueBacksideCluster.getTargetPosition(localization);
            default:
                return null;
        }
    }

    public double getCamX() { return this.latestCamX; }
    public double getCamY() { return this.latestCamY; }
    public double getCamHeading() { return this.latestCamHeading; }

    private int getIndexInArray(int[] array, int targetId) {
        if (array == null) return -1;
        for (int i = 0; i < array.length; i++) {
            if (array[i] == targetId) return i;
        }
        return -1;
    }

    public int getLatestTagId() { return this.latestTagId; }
    public boolean isLatestTagUpsideDown() { return this.latestIsUpsideDown; }
    
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