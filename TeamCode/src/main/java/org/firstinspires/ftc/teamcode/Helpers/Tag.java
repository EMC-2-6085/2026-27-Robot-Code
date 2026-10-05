package org.firstinspires.ftc.teamcode.Helpers;

//all arrays are x, y, z
public class Tag {
    public boolean isSeen;
    public double[] tagPosition = new double[3];
    
    public void setPosition(double tagX, double tagY, double tagZ) {
        //sets the values of the tag, call whenever tag is seen 
        isSeen = true;
        tagPosition[0] = tagX;
        tagPosition[1] = tagY;
        tagPosition[2] = tagZ;
    }
    public void setNotSeen() {
        //call when tag is not seen and isSeen was true
        isSeen = false;
    }

    public double[] getPosition() {
        //returns array of x, y, z  
        return tagPosition;
    }
    public boolean isTagSeen() {
        return isSeen;
    }
}