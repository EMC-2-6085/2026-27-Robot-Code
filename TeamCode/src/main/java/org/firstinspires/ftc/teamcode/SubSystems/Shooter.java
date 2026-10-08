package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.shooterConstants;

public class Shooter {

    public final DcMotor shooterMotor;
    public MotorVelocityPID shooterMotorPID;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        shooterMotorPID = new MotorVelocityPID(
            shooterMotor,
            shooterConstants.ENCODER_CPR,
            shooterConstants.SHOOTER_KP,
            shooterConstants.SHOOTER_KI,
            shooterConstants.SHOOTER_KD,
            shooterConstants.INTEGRAL_LIMIT,
            shooterConstants.ACCEPTABLE_ERROR
        );
    }

    public void prepareShot() {
        setTargetRPM(shooterConstants.SHOOTER_RPM);
    }

    public void spinBackward() {
        setTargetRPM(shooterConstants.UNLOAD_RPM);
    }

    public void stop() {
        setTargetRPM(0);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void setTargetRPM(double rpm) {
        shooterMotorPID.setTargetRPM(rpm);
    }

    public double getTargetRPM() {
        return shooterMotorPID.getTargetRPM();
    }

    public double getRPM() {
        return shooterMotorPID.getRPM();
    }

    public double getError() {
        return shooterMotorPID.getError();
    }

    public double getPower() {
        return shooterMotorPID.getPower();
    }

    public boolean checkAcceptableError() {
        return shooterMotorPID.isAtTargetRPM();
    }

    public void periodic() {
        shooterMotorPID.periodic();
    }
}