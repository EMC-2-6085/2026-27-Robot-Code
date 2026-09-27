package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.driveTrainConstants;

public class Drivetrain {

    public final DcMotor leftRear, rightRear, leftFront, rightFront;

    private final double wheelPowerLarge = driveTrainConstants.WHEEL_POWER_LARGE;
    private double headingP = driveTrainConstants.HEADING_P;

    public Drivetrain(HardwareMap hardwareMap) {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveRobot(double x, double y, double turn, double heading, double targetHeading, boolean moveField) {
        double rotX = x;
        double rotY = y;
        double rotTurn = turn;

        if (moveField) {
            if (Math.abs(turn) < 0.05) {
                double headingError = targetHeading - heading;
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;
                rotTurn = headingError * headingP;
            }

            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            rotX = x * cos - y * sin;
            rotY = x * sin + y * cos;
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotTurn), 1.0);
        double lf = (rotY + rotX + rotTurn) / denominator;
        double lr = (rotY - rotX + rotTurn) / denominator;
        double rf = (rotY - rotX - rotTurn) / denominator;
        double rr = (rotY + rotX - rotTurn) / denominator;

        setPower(lr * wheelPowerLarge, rr * wheelPowerLarge, lf * wheelPowerLarge, rf * wheelPowerLarge);
    }

    public void stop() { setPower(0, 0, 0, 0); }

    public void setPower(double lr, double rr, double lf, double rf) {
        leftRear.setPower(lr);
        rightRear.setPower(rr);
        leftFront.setPower(lf);
        rightFront.setPower(rf);
    }
}