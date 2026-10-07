package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Constants.indexerConstants;

public class Indexer {

    public final DcMotor indexerMotor;
    public MotorVelocityPID indexerMotorPID;

    public Indexer(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.get(DcMotor.class, "indexerMotor");
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        indexerMotorPID = new MotorVelocityPID(
            indexerMotor,
            indexerConstants.ENCODER_CPR,
            indexerConstants.INDEXER_KP,
            indexerConstants.INDEXER_KI,
            indexerConstants.INDEXER_KD,
            indexerConstants.INTEGRAL_LIMIT,
            indexerConstants.ACCEPTABLE_ERROR
        );
    }

    public void spinForward() {
        setTargetRPM(indexerConstants.INDEXER_RPM);
    }

    public void spinBackward() {
        setTargetRPM(-indexerConstants.INDEXER_RPM);
    }

    public void stop() {
        setTargetRPM(0);
    }

    public void setPower(double power) {
        indexerMotor.setPower(power);
    }

    public void setTargetRPM(double rpm) {
        indexerMotorPID.setTargetRPM(rpm);
    }

    public double getTargetRPM() {
        return indexerMotorPID.getTargetRPM();
    }

    public double getRPM() {
        return indexerMotorPID.getRPM();
    }

    public double getError() {
        return indexerMotorPID.getError();
    }

    public double getPower() {
        return indexerMotorPID.getPower();
    }

    public boolean checkAcceptableError() {
        return indexerMotorPID.isAtTargetRPM();
    }

    public void periodic() {
        indexerMotorPID.periodic();
    }
}