package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotboard.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Util.Constants.localizationConstants;

public class Localization {
    private double x = 0;
    private double y = 0;
    private double heading = 0;

    private final GoBildaPinpointDriver pinpoint;

    public Localization(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(localizationConstants.Y_OFFSET, localizationConstants.X_OFFSET); 
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.EncoderResolution.FOUR_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.update(); 
    }
    
    //this is all a bit fucky with mephis x and y being opposite of the pinpoints but it should work
    public void setX(double newX) { 
        this.x = newX;
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, this.y, newX, AngleUnit.RADIANS, this.heading));
    }
    
    public void setY(double newY) { 
        this.y = newY;
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, newY, this.x, AngleUnit.RADIANS, this.heading));
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    
    public Pose2D getVelocity() {
        Pose2D rawVel = pinpoint.getVelocity();
        return new Pose2D(DistanceUnit.METER, rawVel.getY(DistanceUnit.METER), rawVel.getX(DistanceUnit.METER), AngleUnit.RADIANS, rawVel.getHeading(AngleUnit.RADIANS));
    } //might need this at some point idk
    
    public void resetGyro() { 
        pinpoint.recalibrateIMU(); 
    }
    
    public void resetPosition() { 
        setX(0);
        setY(0);
    }

    public void resetEverything() { 
        pinpoint.resetPosAndIMU();
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    } //dont call at the start of teleop

    public void periodic() {
        pinpoint.update();
        Pose2D rawPose = pinpoint.getPosition();
        
        double rawX = rawPose.getX(DistanceUnit.METER);
        double rawY = rawPose.getY(DistanceUnit.METER);
        this.x = rawY;
        this.y = rawX;
        this.heading = rawPose.getHeading(AngleUnit.RADIANS);
    }
}