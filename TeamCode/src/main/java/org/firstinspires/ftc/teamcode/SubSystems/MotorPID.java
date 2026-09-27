package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorPID {

    private final DcMotor motor;

    private double kP;
    private double kI;
    private double kD;
    private double integralLimit;
    private double acceptableError;

    private double targetPosition;
    private double integral = 0;
    private double lastError = 0;
    private long lastErrorChangeTime = 0;
    private long lastLoopTime;

    private boolean powerMode = false;
    private double directPower = 0;

    public MotorPID(DcMotor motor, double kP, double kI, double kD,
                    double integralLimit, double acceptableError) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = integralLimit;
        this.acceptableError = acceptableError;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.targetPosition = 0; 
        this.integral = 0;
        this.lastError = 0;
        this.lastErrorChangeTime = 0;
        this.lastLoopTime = System.currentTimeMillis();
    }

    public void setTarget(double target) {
        this.powerMode = false;
        targetPosition = target; 
        integral = 0;
        lastError = 0;
        lastErrorChangeTime = System.currentTimeMillis();
    }

    public void setPower(double power) {
        this.powerMode = true;
        this.directPower = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(this.directPower);
    }

    public void reset() {
        this.integral = 0;
        this.lastError = 0;
        this.lastErrorChangeTime = System.currentTimeMillis();
        this.lastLoopTime = System.currentTimeMillis();
    }

    public void periodic() {
        if (powerMode) {
            motor.setPower(directPower);
            return;
        }

        double currentPos = motor.getCurrentPosition();
        double error = targetPosition - currentPos;

        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;

        if (dt <= 0 || dt > 0.1) dt = 0.02;

        if (Math.abs(error - lastError) > 1) {
            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
            lastErrorChangeTime = now;
        } else if (now - lastErrorChangeTime > 200) {
            integral = 0;
        }

        double derivative = (error - lastError) / dt;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        output = Math.max(-1, Math.min(1, output));

        if (Math.abs(error) < acceptableError) {
            output = 0;
            motor.setPower(0);
        } else {
            motor.setPower(output);
        }

        lastError = error;
        lastLoopTime = now;
    }

    public double getError() { return lastError; }

    public boolean isAtTarget() {
        if (powerMode) return false;
        return Math.abs(targetPosition - motor.getCurrentPosition()) < acceptableError;
    }

    public double getPosition() { return motor.getCurrentPosition(); }

    public double getTarget() { return targetPosition; }

    public double getPower() { return motor.getPower(); }
    
    public void setkP(double kP) { this.kP = kP; }
    public void setkI(double kI) { this.kI = kI; }
    public void setkD(double kD) { this.kD = kD; }

    public double getkP() { return this.kP; }
    public double getkI() { return this.kI; }
    public double getkD() { return this.kD; }

    public void stop() { 
        powerMode = true;
        directPower = 0;
        motor.setPower(0); 
    }
}