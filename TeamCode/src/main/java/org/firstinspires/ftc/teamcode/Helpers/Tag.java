package org.firstinspires.ftc.teamcode.Helpers;

//all arrays are x,y
public class Tag {
    //TODO: add back checking z incase we see tags from a tipped hive
    public double x, y;
    public boolean isSeen;
    public double[] tagPosition = new double[2];
    
    public void setPosition(double tagX, double tagY) {
        //sets the values of the tag, call whenever tag is seen 
        isSeen = true;
        x = tagX;
        y = tagY;

        tagPosition[0] = x;
        tagPosition[1] = y;
    }
    public void setNotSeen() {
        //call when tag is not seen and isSeen was true
        isSeen = false;
    }

    public double[] getPosition() {
        //returns array of x, y
        return tagPosition;
    }
    public boolean isTagSeen() {
        return isSeen;
    }
}