package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SubSystems.*;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class Robot {

    public final Drivetrain drive;
    public final Vision vision;        
    public final Localization local;   
    public final Lights lights;
    public final IntakeArm intakeArm;
    public final Intake intake;
    public final Indexer indexer;
    public final GlassWindow glassWindow;

    public final MotorPID posPIDLeftRear;
    public final MotorPID posPIDRightRear;
    public final MotorPID posPIDLeftFront;
    public final MotorPID posPIDRightFront;

    public final Mephistopheles mephi;

    public Robot(HardwareMap hardwareMap) {
        drive = new Drivetrain(hardwareMap);
        vision = new Vision(hardwareMap);
        local = new Localization(hardwareMap, vision);
        lights = new Lights(hardwareMap);
        intakeArm = new IntakeArm(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        glassWindow = new GlassWindow(hardwareMap, vision);

        double kP = Constants.driveTrainConstants.WHEEL_P;
        double kI = Constants.driveTrainConstants.WHEEL_I;
        double kD = Constants.driveTrainConstants.WHEEL_D;
        double integralLimit = Constants.driveTrainConstants.INTERGRAL_LIMIT;
        double acceptableError = Constants.driveTrainConstants.ACCEPTABLE_ERROR;

        posPIDLeftRear  = new MotorPID(drive.leftRear,  kP, kI, kD, integralLimit, acceptableError);
        posPIDRightRear = new MotorPID(drive.rightRear, kP, kI, kD, integralLimit, acceptableError);
        posPIDLeftFront = new MotorPID(drive.leftFront, kP, kI, kD, integralLimit, acceptableError);
        posPIDRightFront = new MotorPID(drive.rightFront, kP, kI, kD, integralLimit, acceptableError);

        mephi = new Mephistopheles (
            hardwareMap,
            local,
            posPIDLeftRear,
            posPIDRightRear,
            posPIDLeftFront,
            posPIDRightFront,
            lights
        );
    }
}