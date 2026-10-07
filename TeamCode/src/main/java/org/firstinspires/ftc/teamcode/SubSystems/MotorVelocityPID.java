package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorVelocityPID {

    private final DcMotor motor;
    private final double encoderCPR;

    private double kP;
    private double kI;
    private double kD;
    private double integralLimit;
    private double acceptableError;

    private double targetRPM;
    private double integral = 0;
    private double lastError = 0;
    private long lastErrorChangeTime = 0;
    private long lastLoopTime;

    private double lastPosition = 0;

    private boolean powerMode = false;
    private double directPower = 0;

    public MotorVelocityPID(DcMotor motor, double encoderCPR, double kP, double kI, double kD,
                            double integralLimit, double acceptableError) {
        this.motor = motor;
        this.encoderCPR = encoderCPR;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = integralLimit;
        this.acceptableError = acceptableError;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.targetRPM = 0; 
        this.integral = 0;
        this.lastError = 0;
        this.lastErrorChangeTime = 0;
        this.lastPosition = motor.getCurrentPosition();
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
        motor.setPower(this.directPower);
    }

    public void reset() {
        this.integral = 0;
        this.lastError = 0;
        this.lastErrorChangeTime = System.currentTimeMillis();
        this.lastPosition = motor.getCurrentPosition();
        this.lastLoopTime = System.currentTimeMillis();
    }

    public void periodic() {
        if (powerMode) {
            motor.setPower(directPower);
            return;
        }

        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;

        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double currentPos = motor.getCurrentPosition();

        double currentTicksPerSec = (currentPos - lastPosition) / dt;

        double currentRPM = (currentTicksPerSec * 60.0) / encoderCPR;

        double error = targetRPM - currentRPM;

        if (Math.abs(error - lastError) > 0.5) {
            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
            lastErrorChangeTime = now;
        } else if (now - lastErrorChangeTime > 200) {
            integral = 0;
        }

        double derivative = (error - lastError) / dt;
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        output = Math.max(-1.0, Math.min(1.0, output));

        if (targetRPM == 0 && Math.abs(currentRPM) < acceptableError) {
            motor.setPower(0);
        } else {
            motor.setPower(output);
        }

        lastPosition = currentPos;
        lastError = error;
        lastLoopTime = now;
    }

    public double getError() { return lastError; }

    public boolean isAtTargetRPM() {
        if (powerMode) return false;
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;
        if (dt <= 0) dt = 0.02;
        double currentRPM = (((motor.getCurrentPosition() - lastPosition) / dt) * 60.0) / encoderCPR;
        return Math.abs(targetRPM - currentRPM) < acceptableError;
    }

    public double getRPM() {
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;
        if (dt <= 0) return 0;
        double currentTicksPerSec = (motor.getCurrentPosition() - lastPosition) / dt;
        return (currentTicksPerSec * 60.0) / encoderCPR;
    }

    public double getTargetRPM() { return targetRPM; }

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