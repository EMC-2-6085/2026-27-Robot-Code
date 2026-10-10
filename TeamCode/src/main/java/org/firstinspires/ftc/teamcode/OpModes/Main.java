package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.GlassWindow;
import org.firstinspires.ftc.teamcode.Util.Constants.driveTrainConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Util.Constants.intakeArmConstants;
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
            glassWindow.start(); 
        } catch(Exception ignored) {}
        currentHeading = robot.local.getHeading();

        glassWindow.addCamera("Vision/Webcam 1", "Webcam 1", "4:3");
        glassWindow.addCamera("Vision/Webcam 2", "Webcam 2", "4:3");

        IntakeCommand raiseCommand = new IntakeCommand(
            robot.intakeArm, 
            robot.intake, 
            intakeArmConstants.ARM_UP_RADIANS
        );
        IntakeCommand lowerCommand = new IntakeCommand(
            robot.intakeArm, 
            robot.intake, 
            intakeArmConstants.ARM_DOWN_RADIANS
        );
        IntakeCommand inCommand = new IntakeCommand(
            robot.intakeArm, 
            robot.intake, 
            intakeArmConstants.ARM_IN_RADIANS
        );
        
        ShootCommand shootCommand = new ShootCommand(
            robot.mephi, 
            robot.vision, 
            robot.local, 
            robot.shooter, 
            80
        );
        
        boolean isShooting = false;
        
        waitForStart();

        try {
            while (opModeIsActive()) {
                robot.local.periodic();
                robot.intake.periodic();
                robot.intakeArm.periodic();
                robot.vision.update();

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

                if (gamepad1.backWasPressed()) { 
                    moveField = !moveField; 
                }
                if (gamepad1.yWasPressed()) {
                    robot.local.resetGyro();
                }

                if (gamepad2.rightBumperWasPressed()) {
                    raiseCommand.execute();
                } else if (gamepad2.rightTriggerWasPressed()) {
                    lowerCommand.execute();
                } else if (gamepad2.yWasPressed()) {
                    inCommand.execute();
                }
                
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
                
                telemetry.addData("Movement", moveField ? "Field" : "Robot");
                telemetry.addData("X (m)", robot.local.getX());
                telemetry.addData("Y (m)", robot.local.getY());
                telemetry.addData("Heading (rad)", currentHeading);
                telemetry.addData("Active Cluster", robot.vision.getActiveCluster());
                telemetry.addData("Latest Tag ID", robot.vision.getLatestTagId());
                telemetry.addData("Raw Detections Count", robot.vision.getCamDetections().size());
                telemetry.addData("Latest Raw Tag ID", robot.vision.getLatestTagId());
                double[] targetPose = robot.vision.getActiveTargetPosition(robot.local);
                
                if (targetPose != null) {
                    telemetry.addData("Target X (m)", targetPose[0]);
                    telemetry.addData("Target Y (m)", targetPose[1]);
                    telemetry.addData("Target Heading (rad)", targetPose[2]);
                } else {
                    telemetry.addData("Target Position", "No active hive cluster in view");
                }
                telemetry.update();

                glassWindow.addDouble("Robot/X", robot.local.getX());
                glassWindow.addDouble("Robot/Y", robot.local.getY());
                glassWindow.addGraph("Robot/Heading", robot.local.getHeading(), null, 0.00001);
                glassWindow.addField("Field/Vector Map", robot.local.getX(), robot.local.getY(), robot.local.getHeading());
                glassWindow.addGraph("Battery/Volts", voltSensor.getVoltage(), "V", 0.01);
            }

        } finally {
            if (glassWindow != null) {
                glassWindow.stopServer();
            }   
        }
    }
}