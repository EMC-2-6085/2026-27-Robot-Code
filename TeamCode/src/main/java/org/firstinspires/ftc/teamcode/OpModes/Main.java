package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.GlassWindow;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Constants.driveTrainConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Util.Constants.intakeArmConstants;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ForwardIndexCommand;
import org.firstinspires.ftc.teamcode.Commands.BackwardIndexCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Commands.OhShitCommand;
import java.util.List;


@TeleOp(name = "Main TeleOp")
public class Main extends LinearOpMode {
    
    private GlassWindow glassWindow;
    private double targetHeading;
    private double currentHeading;
    private boolean moveField = true;
    private double x;
    private double y;
    private VoltageSensor voltSensor; 

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        voltSensor = hardwareMap.voltageSensor.get("Control Hub");

        glassWindow = new GlassWindow(hardwareMap, robot.vision);
        try { 
            if (Constants.DEBUG == true) {
                glassWindow.start(); 
            } 
        } catch(Exception ignored) {}
        currentHeading = robot.local.getHeading();

        if (Constants.DEBUG == true) {
            glassWindow.addCamera("Vision/Webcam 1", "Webcam 1", "4:3");
        }
        
        IntakeCommand raiseCommand = new IntakeCommand(robot.intakeArm, robot.intake, intakeArmConstants.ARM_UP_RADIANS);
        IntakeCommand lowerCommand = new IntakeCommand(robot.intakeArm, robot.intake, intakeArmConstants.ARM_DOWN_RADIANS);
        IntakeCommand inCommand = new IntakeCommand(robot.intakeArm, robot.intake,intakeArmConstants.ARM_IN_RADIANS);
        ShootCommand shootCommand = new ShootCommand(robot.mephi, robot.vision, robot.local, robot.shooter, robot.indexer, 80, true);
        //true if red, false if blue
        ForwardIndexCommand forwardIndex = new ForwardIndexCommand(robot.indexer);
        BackwardIndexCommand backwardIndex = new BackwardIndexCommand(robot.indexer);
        OhShitCommand ohShitCommand = new OhShitCommand(robot.intake, robot.intakeArm, robot.indexer,robot.shooter);
        
        boolean isShooting = false;
        
        waitForStart();

        try {
            while (opModeIsActive()) {
                robot.local.periodic();
                robot.intake.periodic();
                robot.intakeArm.periodic();
                robot.indexer.periodic();
                robot.shooter.periodic();
                //robot.vision.update(); camera not wired yet
                
                robot.shooter.prepareShot();

                currentHeading = robot.local.getHeading();
                double turn = gamepad1.right_stick_x;

                if (gamepad1.dpad_up) {
                    y = driveTrainConstants.WHEEL_POWER_TINY;
                } else if (gamepad1.dpad_down) {
                    y = -driveTrainConstants.WHEEL_POWER_TINY;
                } else if (gamepad1.dpad_left) {
                    x = -driveTrainConstants.WHEEL_POWER_TINY;
                } else if (gamepad1.dpad_right) {
                    x = driveTrainConstants.WHEEL_POWER_TINY;
                } else {
                    x = gamepad1.left_stick_x;
                    y = -gamepad1.left_stick_y; 
                }
                if (Math.abs(turn) < 0.05) {
                    if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05) {
                    } else {
                        targetHeading = currentHeading;
                    }
                } else {
                    targetHeading = currentHeading;
                }
                robot.drive.moveRobot(x, y, turn, currentHeading, targetHeading, moveField);

                //toggle movement type
                if (gamepad1.backWasPressed()) { 
                    moveField = !moveField; 
                }
                //reset gyro
                if (gamepad1.yWasPressed()) {
                    robot.local.resetGyro();
                }
                //speed down
                if (gamepad1.leftTriggerWasPressed()) {
                    robot.drive.multiDown();
                }
                //speed up
                if (gamepad1.rightTriggerWasPressed()) {
                    robot.drive.multiUp();
                }
                //intake arm positions
                if (gamepad2.rightBumperWasPressed()) {
                    lowerCommand.execute();
                } else if (gamepad2.rightTriggerWasPressed()) {
                    inCommand.execute();
                } else if (gamepad2.yWasPressed()) {
                    raiseCommand.execute();
                }
                //try to shoot
                if (gamepad2.leftBumperWasPressed()) {
                    shootCommand.end();
                    isShooting = true;
                }
                if (isShooting) {
                    shootCommand.execute();
                    if (shootCommand.isFinished()) {
                        isShooting = false;
                    }
                }
                //expel all balls
                if (gamepad2.backWasPressed()) {
                    ohShitCommand.execute();
                }
                //indexer commands
                if (gamepad2.x) {
                    forwardIndex.execute();
                } else if (gamepad2.b) {
                    backwardIndex.execute();
                } else {
                    forwardIndex.end();
                    backwardIndex.end();
                }
                
                telemetry.addData("Movement", moveField ? "Field" : "Robot");
                telemetry.addData("X (m)", robot.local.getX());
                telemetry.addData("Y (m)", robot.local.getY());
                telemetry.addData("Heading (rad)", currentHeading);
                telemetry.addData("Active Cluster", robot.vision.getActiveCluster());
                telemetry.addData("Tags Visible", robot.vision.getCamDetections().size());
                telemetry.addData("Arm Target (rad)", robot.intakeArm.getTargetPosition());
                telemetry.addData("Arm At Target", robot.intakeArm.checkAcceptableError());
                telemetry.addData("Intake RPM", robot.intake.getRPM());
                telemetry.addData("Intake At Target", robot.intake.checkAcceptableError());
                telemetry.addData("Indexer RPM", robot.indexer.getRPM());
                telemetry.addData("Indexer At Target", robot.indexer.checkAcceptableError());
                telemetry.addData("Shooter RPM", robot.shooter.getRPM());
                telemetry.addData("Shooter At Target", robot.shooter.checkAcceptableError());

                //target position for shooting
                double[] targetPose = robot.vision.getActiveTargetPosition(robot.local);
                if (targetPose != null) {
                    telemetry.addData("Target X (m)", targetPose[0]);
                    telemetry.addData("Target Y (m)", targetPose[1]);
                    telemetry.addData("Target Heading (rad)", targetPose[2]);
                }
                
                //Range and FOM for each tag
                if (Constants.DEBUG == true) {
                     List<AprilTagDetection> currentDetections = robot.vision.getCamDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection instanceof AprilTagSingleDetection) {
                            org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection singleDet = 
                                (org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection) detection;
                            if (singleDet.ftcPose != null) {
                                telemetry.addData("Tag ID " + singleDet.id, 
                                    String.format("Range: %.2fm | FOM: %.1f", 
                                    singleDet.ftcPose.range, 
                                    (double) singleDet.decisionMargin));
                            }
                        }
                    }
                }

                telemetry.update();

                if (Constants.DEBUG == true) {
                    //i love telemetry charlie! i love telemetry!
                    glassWindow.addDouble("Robot/X", robot.local.getX());
                    glassWindow.addDouble("Robot/Y", robot.local.getY());
                    glassWindow.addGraph("Robot/Heading", robot.local.getHeading(), null, 0.0001);
                    glassWindow.addField("Field/Vector Map", robot.local.getX(), robot.local.getY(), robot.local.getHeading());
                    glassWindow.addGraph("Battery/Volts", voltSensor.getVoltage(), "V", 0.01);
                    glassWindow.addString("Vision/Active Cluster", robot.vision.getActiveCluster());
                    glassWindow.addDouble("Vision/Tags Visible", robot.vision.getCamDetections().size());
                    glassWindow.addDouble("Arm/Target (rad)", robot.intakeArm.getTargetPosition());
                    glassWindow.addDouble("Arm/Position (rad)", robot.intakeArm.intakeArmPID.getPosition() / intakeArmConstants.RADIANS_TO_TICKS);
                    glassWindow.addBoolean("Arm/At Target", robot.intakeArm.checkAcceptableError());
                    glassWindow.addDouble("Intake/RPM", robot.intake.getRPM());
                    glassWindow.addDouble("Intake/Target RPM", robot.intake.getTargetRPM());
                    glassWindow.addBoolean("Intake/At Target", robot.intake.checkAcceptableError());
                    glassWindow.addDouble("Indexer/RPM", robot.indexer.getRPM());
                    glassWindow.addDouble("Indexer/Target RPM", robot.indexer.getTargetRPM());
                    glassWindow.addBoolean("Indexer/At Target", robot.indexer.checkAcceptableError());
                    glassWindow.addDouble("Shooter/RPM", robot.shooter.getRPM());
                    glassWindow.addDouble("Shooter/Target RPM", robot.shooter.getTargetRPM());
                    glassWindow.addBoolean("Shooter/At Target", robot.shooter.checkAcceptableError());
                }
            }

        } finally {
            if (Constants.DEBUG == true && glassWindow != null) {
                glassWindow.stopServer();
            }   
        }
    }
}