package org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.SubSystems.Localization;
import org.firstinspires.ftc.teamcode.Util.Constants.shooterConstants;

//all arrays are x, y, z or x, y, z, heading
public class TagCluster {
    public double[] tag1 = new double[3];
    public double[] tag2 = new double[3];
    public double[] tag3 = new double[3];
    public double[] tag4 = new double[3];
    public double[][] cluster = new double[][] {tag1, tag2, tag3, tag4};
    public int offsetMultipler = 1;

    public TagCluster(int multiplier) {
        this.offsetMultiplier = multiplier;
    }

    public void setCluster(double[] newTag1, double[] newTag2, double[] newTag3, double[] newTag4) {
        //needed to set values of everything in the cluster
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
        //returns long ahh 2d array of all the tag data in the cluster
        //dont know why this would be needed
        return cluster;
    }
    public double[] getAverage() {
        //returns average of all tag distances
        double sumX = 0;
        double sumY = 0;
        int seenCount = 0;
        double[] clusterAverage = new double[2];

        for (double[] tag : cluster) {
            //check that tag exists and has data
            if (tag != null && (tag[0] != 0 || tag[1] != 0)) {
                sumX += tag[0];
                sumY += tag[1];
                seenCount++;
            }
        }
        //avoid division by zero
        if (seenCount > 0) {
            clusterAverage[0] = -(sumX / seenCount);
            clusterAverage[1] = sumY / seenCount;
        }
        return clusterAverage;
    }
    public double[] getTargetPosition(Localization localization) {
        //finds mephi's target for shooting
        //to robot center
        double[] currentPosition = new double[2];
        currentPosition[0] = localization.getX();
        currentPosition[1] = localization.getY();
        
        double[] offset = getAverage();

        double[] newPosition = new double[3];
        newPosition[0] = (currentPosition[0] + (offset[0] * 0.00254 * -offsetMultipler)) + shooterConstants.CAMERA_OFFSET[0] * -offsetMultipler; //convert offset to meters and radians from inches and degrees
        newPosition[1] = (currentPosition[1] + (offset[1] * 0.00254 * -offsetMultipler) + (shooterConstants.SHOOT_DISTANCE * offsetMultipler)) + shooterConstants.CAMERA_OFFSET[1] * -offsetMultipler;

        if (offsetMultipler == -1) {
            newPosition[2] = 0;
        } else {
            newPosition[2] = 3.14159;
        }
        return newPosition;
    }

    public void invertOffset() {
        //for second set of hives' needed y distance
        offsetMultipler = -offsetMultipler;
    }
}