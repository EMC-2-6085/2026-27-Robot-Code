package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.GlassWindow;
import org.firstinspires.ftc.teamcode.Util.Constants.driveTrainConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name = "Mephi Drivetrain")
public class Drivetrain extends LinearOpMode {
    
    private GlassWindow glassWindow;
    private double targetHeading;
    private double currentHeading;
    private boolean moveField = true;

    private VoltageSensor voltSensor; 

    @Override
    public void runOpMode() throws InterruptedException {

        voltSensor = hardwareMap.voltageSensor.get("Control Hub");

        Robot robot = new Robot(hardwareMap);
        glassWindow = new GlassWindow(hardwareMap, robot.vision);
        try { 
            glassWindow.start(); 
        } catch(Exception ignored) {}
        robot.local.resetPosition();
        robot.local.setX(0.2159);
        robot.local.setY(0.2159);
        currentHeading = 0;

        glassWindow.addCamera("Vision/Webcam 1", "Webcam 1", "4:3");
        glassWindow.addCamera("Vision/Webcam 2", "Webcam 2", "4:3");
        
        glassWindow.addString("Lights/Light Color", "green", true);
        robot.lights.greenOn();

        while (!isStarted() && !isStopRequested()) {
            robot.vision.update();
            currentHeading = robot.local.getHeading();
            
            telemetry.addData("X (m)", robot.local.getX());
            telemetry.addData("Y (m)", robot.local.getY());
            telemetry.addData("Heading (rad)", currentHeading);
            telemetry.update();
        }

        waitForStart();

        try {
            while (opModeIsActive()) {
                robot.vision.update();
                robot.local.periodic();
                
                currentHeading = robot.local.getHeading();
                
                if (gamepad1.backWasPressed()) { 
                    moveField = !moveField; 
                }
                if (gamepad1.yWasPressed()) {
                    robot.local.resetGyro();
                }
    
                double x = 0;
                double y = 0;
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
                
                List<AprilTagDetection> currentDetections = robot.vision.getCamDetections();
    
                telemetry.addData("Movement", moveField ? "Field" : "Robot");
                telemetry.addData("X (m)", robot.local.getX());
                telemetry.addData("Y (m)", robot.local.getY());
                telemetry.addData("Heading (rad)", currentHeading);
                telemetry.addData("Detections", currentDetections.size());
                telemetry.update();

                glassWindow.addInt("Vision/Tag Detections", currentDetections.size());
                glassWindow.addDouble("Robot/X", robot.local.getX());
                glassWindow.addDouble("Robot/Y", robot.local.getY());
                glassWindow.addGraph("Robot/Heading", robot.local.getHeading(), null, 0.00001);
                glassWindow.addGraph("Battery/Volts", voltSensor.getVoltage(), "V", 0.1);
                
                String lightColor = glassWindow.getString("Lights/Light Color", "white");
                if ("red".equals(lightColor)) {
                    robot.lights.redOn();
                } else if ("green".equals(lightColor)) {
                    robot.lights.greenOn();
                } else if ("blue".equals(lightColor)) {
                    robot.lights.blueOn();
                } else if ("white".equals(lightColor)) {
                    robot.lights.whiteOn();
                } else {
                    robot.lights.setWhite(false);
                    robot.lights.setRed(false);
                    robot.lights.setGreen(false);
                    robot.lights.setBlue(false);
                }
                
                glassWindow.addField("Field/Vector Map", robot.local.getX(), robot.local.getY(), robot.local.getHeading());
            }
        } finally {
            if (glassWindow != null) {
                glassWindow.stopServer();
            }
        }
    }
}