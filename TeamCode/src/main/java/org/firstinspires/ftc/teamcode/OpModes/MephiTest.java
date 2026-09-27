package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.GlassWindow;
import org.firstinspires.ftc.teamcode.SubSystems.Mephistopheles.MephistophelesPath;
import org.firstinspires.ftc.teamcode.SubSystems.Mephistopheles.PathConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@Autonomous(name = "Mephi go vroom vroom")
public class MephiTest extends LinearOpMode {
    
    private Robot robot;
    private GlassWindow glassWindow;
    private MephistophelesPath pathFollower;
    private VoltageSensor voltSensor; 

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        voltSensor = hardwareMap.voltageSensor.get("Control Hub");
        
        glassWindow = new GlassWindow(hardwareMap, robot.vision);
        try { 
            glassWindow.start(); 
        } catch(Exception ignored) {}

        robot.local.resetPosition();
        robot.local.setX(0.2159);
        robot.local.setY(0.2159);

        PathConfig config = new PathConfig("TestPath.gcode");

        pathFollower = new MephistophelesPath(
                robot.mephi,
                robot.local,
                config
        );
        
        glassWindow.addCamera("Vision/Webcam 1", "Webcam 1", "4:3");
        glassWindow.addCamera("Vision/Webcam 2", "Webcam 2", "4:3");
        
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive() && !isStopRequested()) {
                
                robot.vision.update();
                robot.local.periodic();
        
                boolean completed = pathFollower.follow();
                
                List<AprilTagDetection> currentDetections = robot.vision.getCamDetections();
                
                telemetry.addData("X", robot.local.getX());
                telemetry.addData("Y", robot.local.getY());
                telemetry.addData("Heading", robot.local.getHeading());
                telemetry.addData("Path Done", completed);
                telemetry.update();
                
                glassWindow.addInt("Vision/Tag Detections", currentDetections.size());
                
                glassWindow.addDouble("Robot/X", robot.local.getX());
                glassWindow.addDouble("Robot/Y", robot.local.getY());
                glassWindow.addGraph("Robot/Heading", robot.local.getHeading(), null, 0.00001);

                glassWindow.addGraph("Battery/Volts", voltSensor.getVoltage(), "V", 0.1);
                
                glassWindow.addField("Field/Vector Map", robot.local.getX(), robot.local.getY(), robot.local.getHeading());

                if (completed) {
                    robot.mephi.stop();
                    break;
                }
            }
        } finally {
            if (glassWindow != null) {
                glassWindow.stopServer();
            }
        }
    }
}