package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Constants.localizationConstants;

public class Localization {
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    private final DcMotor leftRear, rightRear, leftFront, rightFront;
    private final IMU imu;
    private final Vision vision; 

    private boolean useEncoders = localizationConstants.USE_ENCODER;
    private boolean useVision = localizationConstants.USE_VISION;

    private int lastLR, lastRR, lastLF, lastRF;
    private double forwardDelta = 0;
    private double strafeDelta = 0;
    private final double distancePerTick;

    private final double encoderWeight = localizationConstants.ENCODER_WEIGHT;
    private final double cameraWeight = localizationConstants.CAMERA_WEIGHT;

    private long resetTimestamp = 0;
    private static final long VISION_LOCKOUT_MS = 500;

    public Localization(HardwareMap hardwareMap, Vision vision) {
        this.vision = vision;

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        distancePerTick = Math.PI * localizationConstants.WHEEL_DIAMETER
                        / localizationConstants.TICKS_PER_REV
                        * localizationConstants.GEAR_RATIO;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);
        resetPosition();
    }

    public void resetEncoders() {
        lastLR = leftRear.getCurrentPosition();
        lastRR = rightRear.getCurrentPosition();
        lastLF = leftFront.getCurrentPosition();
        lastRF = rightFront.getCurrentPosition();
    }

    private void updateEncoders() {
        int currentLR = leftRear.getCurrentPosition();
        int currentRR = rightRear.getCurrentPosition();
        int currentLF = leftFront.getCurrentPosition();
        int currentRF = rightFront.getCurrentPosition();

        int dLR = currentLR - lastLR;
        int dRR = currentRR - lastRR;
        int dLF = currentLF - lastLF;
        int dRF = currentRF - lastRF;

        lastLR = currentLR;
        lastRR = currentRR;
        lastLF = currentLF;
        lastRF = currentRF;

        double LR = dLR * distancePerTick;
        double RR = dRR * distancePerTick;
        double LF = dLF * distancePerTick;
        double RF = dRF * distancePerTick;

        forwardDelta = (LF + RF + LR + RR) / 4.0;
        strafeDelta  = (-LF + RF + LR - RR) / 4.0;
    }

    private void updateIMU() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void periodic() {
        updateIMU();

        double stepForward = 0.0;
        double stepStrafe = 0.0;
        double totalRelativeWeight = 0.0;

        if (useEncoders) {
            updateEncoders();
            stepForward += forwardDelta * encoderWeight;
            stepStrafe += strafeDelta * encoderWeight;
            totalRelativeWeight += encoderWeight;
        }

        if (totalRelativeWeight > 0) {
            stepForward /= totalRelativeWeight;
            stepStrafe /= totalRelativeWeight;
        }

        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        double deltaX = stepForward * sin + stepStrafe * cos; 
        double deltaY = stepForward * cos - stepStrafe * sin; 

        boolean visionLockedOut = (System.currentTimeMillis() - resetTimestamp) < VISION_LOCKOUT_MS;

        if (useVision && !visionLockedOut && vision.getCamDetections().size() > 0) { 
            double camPoseX = vision.getCamX();
            double camPoseY = vision.getCamY();

            double relativeWeightSum = (useEncoders ? encoderWeight : 0);
            double totalWeight = relativeWeightSum + cameraWeight;

            if (totalWeight > 0) {
                this.x = ((relativeWeightSum * (this.x + deltaX)) + (cameraWeight * camPoseX)) / totalWeight;
                this.y = ((relativeWeightSum * (this.y + deltaY)) + (cameraWeight * camPoseY)) / totalWeight;
            }
        } else {
            this.x += deltaX;
            this.y += deltaY;
        }
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public double getCamX() { return vision.getCamX(); }
    public double getCamY() { return vision.getCamY(); }
    public double getCamHeading() { return vision.getCamHeading(); }
    public int getCamDetections() { return vision.getCamDetections().size(); }
    
    public void resetGyro() { imu.resetYaw(); }
    
    public void resetPosition() { 
        x = 0; 
        y = 0; 
        resetGyro(); 
        resetEncoders();
        resetTimestamp = System.currentTimeMillis();
    }
    
    public void setX(double newX) { x = newX; }
    public void setY(double newY) { y = newY; }
    
    public static class TagPose {
        public double x, y, heading;
        public double offsetX, offsetY, offsetHeading;
        public double offsetPitch, offsetRoll;

        public TagPose(double x, double y, double heading,
                       double offsetX, double offsetY, double offsetHeading,
                       double offsetPitch, double offsetRoll) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
            this.offsetHeading = offsetHeading;
            this.offsetPitch = offsetPitch;
            this.offsetRoll = offsetRoll;
        }
    }
}