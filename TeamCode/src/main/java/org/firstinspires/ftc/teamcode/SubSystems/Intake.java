package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.intakeConstants;

public class Intake {

    public final DcMotor intakeMotor;
    public MotorVelocityPID intakeMotorPID;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intakeMotorPID = new MotorVelocityPID(
            intakeMotor,
            intakeConstants.ENCODER_CPR,
            intakeConstants.INTAKE_KP,
            intakeConstants.INTAKE_KI,
            intakeConstants.INTAKE_KD,
            intakeConstants.INTEGRAL_LIMIT,
            intakeConstants.ACCEPTABLE_ERROR
        );
    }

    public void intake() {
        setTargetRPM(intakeConstants.INTAKE_RPM);
    }

    public void outtake() {
        setTargetRPM(-intakeConstants.INTAKE_RPM);
    }

    public void stop() {
        setTargetRPM(0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setTargetRPM(double rpm) {
        intakeMotorPID.setTargetRPM(rpm);
    }

    public double getTargetRPM() {
        return intakeMotorPID.getTargetRPM();
    }

    public double getRPM() {
        return intakeMotorPID.getRPM();
    }

    public double getError() {
        return intakeMotorPID.getError();
    }

    public double getPower() {
        return intakeMotorPID.getPower();
    }

    public boolean checkAcceptableError() {
        return intakeMotorPID.isAtTargetRPM();
    }

    public void periodic() {
        intakeMotorPID.periodic();
    }
}