package org.firstinspires.ftc.teamcode.Helpers;

import org.firstinspires.ftc.teamcode.SubSystems.Localization;
import org.firstinspires.ftc.teamcode.Util.Constants.shooterConstants;

public class TagCluster {
    public double[] tag1 = new double[3];
    public double[] tag2 = new double[3];
    public double[] tag3 = new double[3];
    public double[] tag4 = new double[3];
    public double[][] cluster = new double[][] {tag1, tag2, tag3, tag4};
    public int offsetMultipler = 1;
    private double targetHeading = 3.14159;

    public TagCluster(int multiplier) {
        this.offsetMultipler = multiplier;
    }

    public void setTargetHeading(double heading) {
        this.targetHeading = heading;
    }

    public void setCluster(double[] newTag1, double[] newTag2, double[] newTag3, double[] newTag4) {
        if (newTag1 != null) tag1 = newTag1;
        if (newTag2 != null) tag2 = newTag2;
        if (newTag3 != null) tag3 = newTag3;
        if (newTag4 != null) tag4 = newTag4;

        cluster[0] = tag1;
        cluster[1] = tag2;
        cluster[2] = tag3;
        cluster[3] = tag4;
    }

    public void clearCluster() {
        tag1 = new double[3];
        tag2 = new double[3];
        tag3 = new double[3];
        tag4 = new double[3];
        cluster[0] = tag1;
        cluster[1] = tag2;
        cluster[2] = tag3;
        cluster[3] = tag4;
    }

    public double[][] getCluster() {
        return cluster;
    }
    
    public double[] getAverage() {
        double sumX = 0;
        double sumY = 0;
        int seenCount = 0;
        double[] clusterAverage = new double[2];

        for (double[] tag : cluster) {
            if (tag != null && (tag[0] != 0 || tag[1] != 0)) {
                sumX += tag[0];
                sumY += tag[1];
                seenCount++;
            }
        }
        if (seenCount > 0) {
            clusterAverage[0] = -(sumX / seenCount);
            clusterAverage[1] = sumY / seenCount;
        }
        return clusterAverage;
    }
    
    public double[] getTargetPosition(Localization localization) {
        double currentX = localization.getX();
        double currentY = localization.getY();
        
        double[] offset = getAverage();

        double tagGlobalX = currentX + (offset[0] * -offsetMultipler);
        double tagGlobalY = currentY + (offset[1] * -offsetMultipler);

        double targetX = tagGlobalX - (shooterConstants.CAMERA_OFFSET[0] * offsetMultipler);
        double targetY = tagGlobalY + (shooterConstants.SHOOT_DISTANCE * offsetMultipler) - (shooterConstants.CAMERA_OFFSET[1] * offsetMultipler);

        double[] newPosition = new double[3];
        newPosition[0] = targetX;
        newPosition[1] = targetY;
        newPosition[2] = this.targetHeading;

        return newPosition;
    }

    public void invertOffset() {
        offsetMultipler = -offsetMultipler;
    }
}