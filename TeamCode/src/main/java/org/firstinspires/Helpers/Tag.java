package org.firstinspires.ftc.teamcode.Helpers;

//all arrays are x,y
public class Tag {

    public double x, y;
    public double[] tagPosition = new double[2];

    public double[] getPosition() {
        //returns array of x, y
        return tagPosition;
    }
    public void setPosition(double tagX, double tagY) {
        //sets the values of the tag, call whenever tag is seen
        x = tagX;
        y = tagY;

        tagPosition[0] = x;
        tagPosition[1] = y;
    }
}