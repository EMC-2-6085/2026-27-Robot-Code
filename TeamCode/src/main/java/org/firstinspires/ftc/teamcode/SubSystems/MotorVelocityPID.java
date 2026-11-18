package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorVelocityPID {

    private final DcMotor motor;
    private final DcMotorEx motorEx;
    private final double encoderCPR;

    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double integralLimit;
    private double acceptableError;

    private double targetRPM;
    private double integral = 0;
    private double lastError = 0;
    private long lastErrorChangeTime = 0;
    private long lastLoopTime;

    private boolean powerMode = false;
    private double directPower = 0;

    public MotorVelocityPID(DcMotor motor, double encoderCPR, double kP, double kI, double kD, double kV,
                            double integralLimit, double acceptableError) {
        this.motor = motor;
        if (motor instanceof DcMotorEx) {
            this.motorEx = (DcMotorEx) motor;
        } else {
            this.motorEx = null;
        }
        this.encoderCPR = encoderCPR;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.integralLimit = integralLimit;
        this.acceptableError = acceptableError;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.targetRPM = 0; 
        this.integral = 0;
        this.lastError = 0;
        this.lastErrorChangeTime = 0;
        this.lastLoopTime = System.currentTimeMillis();
    }

    public void setTargetRPM(double rpm) {
        this.powerMode = false;
        targetRPM = rpm; 
        integral = 0;
        lastError = 0;
        lastErrorChangeTime = System.currentTimeMillis();
    }

    public void setPower(double power) {
        this.powerMode = true;
        this.directPower = Math.max(-1.0, Math.min(1.0, power));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(directPower);
            return;
        }

        if (targetRPM == 0) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;
        if (dt <= 0) dt = 0.02;

        double currentTicksPerSec = (motorEx != null) ? motorEx.getVelocity() : 0;
        double currentRPM = (currentTicksPerSec * 60.0) / encoderCPR;

        double error = targetRPM - currentRPM;
        double derivative = (error - lastError) / dt;

        double feedForward = targetRPM * kV;

        double output = feedForward + (kP * error) + (kD * derivative);

        output = Math.max(-1.0, Math.min(1.0, output));

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(output);

        lastError = error;
        lastLoopTime = now;
    }

    public double getError() { return lastError; }

    public boolean isAtTargetRPM() {
        if (powerMode) return false;
        double currentTicksPerSec = (motorEx != null) ? motorEx.getVelocity() : 0;
        double currentRPM = (currentTicksPerSec * 60.0) / encoderCPR;
        return Math.abs(targetRPM - currentRPM) < acceptableError;
    }

    public double getRPM() {
        double currentTicksPerSec = (motorEx != null) ? motorEx.getVelocity() : 0;
        return (currentTicksPerSec * 60.0) / encoderCPR;
    }

    public double getTargetRPM() { return targetRPM; }
    public double getPower() { return motor.getPower(); }
    
    public void setkP(double kP) { this.kP = kP; }
    public void setkI(double kI) { this.kI = kI; }
    public void setkD(double kD) { this.kD = kD; }
    public void setkV(double kV) { this.kV = kV; }

    public double getkP() { return this.kP; }
    public double getkI() { return this.kI; }
    public double getkD() { return this.kD; }
    public double getkV() { return this.kV; }

    public void stop() { 
        powerMode = true;
        directPower = 0;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0); 
    }
}