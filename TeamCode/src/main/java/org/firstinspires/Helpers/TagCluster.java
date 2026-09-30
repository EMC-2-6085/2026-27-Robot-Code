package org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.SubSystems.Localization;

//all arrays are x,y,yaw/heading
public class TagCluster {
    public static Tag tag1 = new Tag();
    public static Tag tag2 = new Tag();
    public static Tag tag3 = new Tag();
    public static Tag tag4 = new Tag();

    public static double[][] cluster = new double[4][3];
    public int offsetMultipler = 1;

    public static void setCluster(double[] newTag1, double[] newTag2, double[] newTag3, double[] newTag4) {
        //needed to set values of everything in the cluster
        tag1 = newTag1;
        tag2 = newTag2;
        tag3 = newTag3;
        tag4 = newTag4;
        cluster[1] = tag1;
        cluster[2] = tag2;
        cluster[3] = tag3;
        cluster[4] = tag4;
    }

    public double[] getCluster() {
        //returns long ahh 2d array of all the tags in the cluster
        return cluster;
    }
    public double[] getAverage() {
        //average of all tag distances
        final double[] clusterAverage;
        final double clusterX = (tag1[1] + tag2[1] + tag3[1] + tag4[1]) / 4;
        final double clusterY = (tag1[2] + tag2[2] + tag3[2] + tag4[2]) / 4;
        final double clusterYaw = (tag1[3] + tag2[3] + tag3[3] + tag4[3]) / 4;
        clusterAverage[1] = -clusterX;
        clusterAverage[2] = clusterY;
        clusterAverage[3] = -clusterYaw;
        return clusterAverage;
    }

    public double[] getTargetPosition() {
        //finds mephi's target for shooting
        double[] currentPosition = new double[3];
        currentPosition[1] = Localization.getX();
        currentPosition[2] = Localization.getY();
        currentPosition[3] = Localization.getHeading();
        
        double[] offset = getAverage();
        double[] newPosition = new double[3];
        newPosition[1] = currentPosition[1] + (offset[1] * 0.00254); //convert offset to meters and radians from inches and degrees
        newPosition[2] = currentPosition[2] + (offset[2] * 0.00254); //need to find/add needed distance (distance * offsetMultiplier)
        newPosition[3] = currentPosition[3] + (offset[3] * 0.0174533);
        return newPosition;
    }

    public void invertOffset() {
        //for second set of hives needed y distance
        offsetMultipler = -offsetMultipler;
    }
}