package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Util.Constants.intakeArmConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class IntakeCommand {
    private IntakeArm intakeArm;
    private Intake intake;
    private double endRadians;

    public IntakeCommand(IntakeArm intakeArm, Intake intake, double endRadians) {
        this.intakeArm = intakeArm;
        this.intake = intake;
        this.endRadians = endRadians;
    }

    public void initialize() {
        intakeArm.setArmTargetPosition(endRadians);
    }

    public void execute() {
        initialize();

        if (endRadians == intakeArmConstants.ARM_DOWN_RADIANS){
            intake.intake();
        }else{
            intake.stop();
        }
    }
    
    public void end() {
        intake.stop();
        intakeArm.armUp();
    }

    public boolean isFinished() {
        return intakeArm.checkAcceptableError();
    }
}