package org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.SubSystems.Localization;
import org.firstinspires.ftc.teamcode.Util.Constants.shooterConstants;

//all arrays are x, y or x, y, heading
public class TagCluster {
    public static double[] tag1, tag2, tag3, tag4 = new double[2];
    public static double[][] cluster = new double[4][2];
    public static int offsetMultipler = 1;

    public static void setCluster(double[] newTag1, double[] newTag2, double[] newTag3, double[] newTag4) {
        //needed to set values of everything in the cluster
        tag1 = newTag1;
        tag2 = newTag2;
        tag3 = newTag3;
        tag4 = newTag4;
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
        //average of all tag distances
        final double[] clusterAverage = new double[2];
        final double clusterX = (tag1[0] + tag2[0] + tag3[0] + tag4[0]) / 4;
        final double clusterY = (tag1[1] + tag2[1] + tag3[1] + tag4[1]) / 4; //TODO: use for loop for tags seen when finding average
        clusterAverage[0] = -clusterX;
        clusterAverage[1] = clusterY;
        return clusterAverage;
    }

    public double[] getTargetPosition(Localization localization) {
        //finds mephi's target for shooting
        //centers to camera lens not robot!
        double[] currentPosition = new double[2];
        currentPosition[0] = localization.getX();
        currentPosition[1] = localization.getY();
        
        double[] offset = getAverage();
        double[] newPosition = new double[3];
        newPosition[0] = currentPosition[0] + (offset[0] * 0.00254 * -offsetMultipler); //convert offset to meters and radians from inches and degrees
        newPosition[1] = currentPosition[1] + (offset[1] * 0.00254 * -offsetMultipler) + (shooterConstants.SHOOT_DISTANCE * offsetMultipler);

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