package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Mephistopheles;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Localization;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Indexer;
import org.firstinspires.ftc.teamcode.Util.Constants.shooterConstants;
import java.util.List;

public class ShootCommand {
    private Mephistopheles mephi;
    private Vision vision;
    private Localization localization;
    private Shooter shooter;
    private Indexer indexer;
    private int speedPct;
    private boolean isRed;
    
    private double[] targetPose = null;
    private boolean finished = false;
    private boolean shotPrepared = false;

    public ShootCommand(Mephistopheles mephi, Vision vision, Localization localization, Shooter shooter, Indexer indexer, int speedPct, boolean isRed) {
        this.mephi = mephi;
        this.vision = vision;
        this.localization = localization;
        this.shooter = shooter;
        this.indexer = indexer;
        this.speedPct = speedPct;
        this.isRed = isRed;
    }

    public void initialize() {
        List<AprilTagDetection> detections = vision.getCamDetections();
        
        boolean visionReliable = false;
        if (detections != null && detections.size() > 2) {
            boolean highConfidence = true;
            for (AprilTagDetection detection : detections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                    if (singleDet.decisionMargin < 75) {
                        highConfidence = false;
                        break;
                    }
                }
            }
            if (highConfidence) {
                visionReliable = true;
            }
        }

        if (visionReliable) {
            targetPose = vision.getActiveTargetPosition(localization);
        }

        if (!visionReliable || targetPose == null) {
            double currentX = localization.getX();
            double currentY = localization.getY();

            if (currentY > 1.3288 && currentY < 2.3288) {
                targetPose = null;
                finished = true; 
                shotPrepared = false;
                return;
            }

            double targetHeading;
            double targetX;
            double targetY;

            double goalYAudience = 0.0;     
            double goalYBackside = 3.6576;  

            boolean isBackside = (currentY >= 2.3288);
            double goalY = isBackside ? goalYBackside : goalYAudience;

            if (isBackside) {
                targetHeading = 3.14; 
                targetY = goalY - shooterConstants.SHOOT_DISTANCE;
            } else {
                targetHeading = 0.0;
                targetY = goalY + shooterConstants.SHOOT_DISTANCE;
            }
            
            if (this.isRed) {
                targetX = 1.524;
            } else {
                targetX = 2.1336;
            }

            targetPose = new double[] { targetX, targetY, targetHeading };
        }

        finished = false;
        shotPrepared = false;
    }

    public void execute() {
        if (targetPose == null) {
            if (finished) return;
            initialize();
        }

        if (targetPose != null) {
            finished = mephi.goToPosition(targetPose[0], targetPose[1], targetPose[2], speedPct, "G01");
            if (finished && !shotPrepared) {
                shooter.prepareShot();
                indexer.spinForward();
                shotPrepared = true;
            }
        } else {
            finished = true;
        }
    }
    
    public void end() {
        targetPose = null;
        finished = false;
        shotPrepared = false;
    }

    public boolean isFinished() {
        return finished;
    }
}