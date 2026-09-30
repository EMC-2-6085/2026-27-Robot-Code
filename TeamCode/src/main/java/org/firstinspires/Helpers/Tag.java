package org.firstinspires.ftc.teamcode.Helpers;

//all arrays are x,y,yaw/heading
public class Tag {

    public double x, y, yaw;
    public double[] tagPosition = new double[3];

    public double[] getPosition() {
        //returns array of x, y, yaw
        return tagPosition;
    }
    public void setPosition(double tagX, double tagY, double tagYaw) {
        //sets the values of the tag, call whenever tag is seen
        x = tagX;
        y = tagY;
        yaw = tagYaw;

        tagPosition[1] = x;
        tagPosition[2] = y;
        tagPosition[3] = yaw;
    }
}