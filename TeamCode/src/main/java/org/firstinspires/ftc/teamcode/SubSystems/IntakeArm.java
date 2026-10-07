package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.intakeArmConstants;

public class IntakeArm {

    public final DcMotor intakeArm;
    public MotorPID intakeArmPID;

    public IntakeArm(HardwareMap hardwareMap) {
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmPID = new MotorPID(intakeArm,
            intakeArmConstants.ARM_KP, intakeArmConstants.ARM_KI, intakeArmConstants.ARM_KD,
            intakeArmConstants.INTEGRAL_LIMIT, intakeArmConstants.ACCEPTABLE_ERROR);
    }

    public void armUp() {
        setArmTargetPosition(intakeArmConstants.ARM_UP_RADIANS);
    }

    public void armDown() {
        setArmTargetPosition(intakeArmConstants.ARM_DOWN_RADIANS);
    }

    public void armIn() {
        setArmTargetPosition(intakeArmConstants.ARM_IN_RADIANS);
    }

    public void setArmTargetPosition(double target) {
        intakeArmPID.setTarget(toEncoderTicks(target));
    }

    public boolean checkAcceptableError() {
        return intakeArmPID.isAtTarget();
    }

    public void setTargetToCurrentPos() {
        intakeArmPID.setTarget(intakeArmPID.getPosition());
    }

    public double toEncoderTicks(double radians) {
        return radians * intakeArmConstants.RADIANS_TO_TICKS;
    }

    public void periodic() {
        intakeArmPID.periodic();
    }
}