package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.SubSystems.Localization.TagPose;
import org.firstinspires.ftc.teamcode.Util.Constants.mephiConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Mephistopheles {

    private final Localization localization;
    private final MotorPID leftRearPID, rightRearPID, leftFrontPID, rightFrontPID;
    private final Lights lights;

    private long targetStartTime = 0;
    private double lastTargetX = Double.NaN;
    private double lastTargetY = Double.NaN;

    private double lastHeadingError = 0;
    private long lastHeadingTime = System.currentTimeMillis();

    public Mephistopheles(HardwareMap hardwareMap,
                          Localization localization,
                          MotorPID leftRearPID,
                          MotorPID rightRearPID,
                          MotorPID leftFrontPID,
                          MotorPID rightFrontPID,
                          Lights lights) {
        this.localization = localization;
        this.leftRearPID = leftRearPID;
        this.rightRearPID = rightRearPID;
        this.leftFrontPID = leftFrontPID;
        this.rightFrontPID = rightFrontPID;
        this.lights = lights;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public boolean goToPosition(double targetX, double targetY, double targetHeading, int speedPct) {

        if (targetX != lastTargetX || targetY != lastTargetY) {
            targetStartTime = System.currentTimeMillis();
            lastTargetX = targetX;
            lastTargetY = targetY;
            lastHeadingError = normalizeAngle(targetHeading - localization.getHeading());
            lastHeadingTime = System.currentTimeMillis();
        }

        double dx = targetX - localization.getX();
        double dy = targetY - localization.getY();
        double distance = Math.hypot(dx, dy);

        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastHeadingTime) / 1000.0;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double headingError = normalizeAngle(targetHeading - localization.getHeading());
        double headingDerivative = (headingError - lastHeadingError) / dt;

        double kP_turn = mephiConstants.TURN_SCALE; 
        double kD_turn = mephiConstants.TURN_D;

        double rotationPower = (headingError * kP_turn) + (headingDerivative * kD_turn);

        lastHeadingError = headingError;
        lastHeadingTime = currentTime;

        double heading = localization.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double forwardError = dx * sin + dy * cos;
        double strafeError  = -(dx * cos - dy * sin);

        double forwardPower  = forwardError * mephiConstants.FORWARD_SCALE;
        double strafePower   = strafeError * mephiConstants.STRAFE_SCALE;

        double leftFrontPower  = forwardPower + strafePower + rotationPower;
        double leftRearPower   = forwardPower - strafePower + rotationPower;
        double rightFrontPower = forwardPower - strafePower - rotationPower;
        double rightRearPower  = forwardPower + strafePower - rotationPower;

        double speedMultiplier = speedPct / 100.0;
        leftFrontPower *= speedMultiplier;
        leftRearPower  *= speedMultiplier;
        rightFrontPower*= speedMultiplier;
        rightRearPower *= speedMultiplier;

        leftFrontPower  = Math.max(-1.0, Math.min(1.0, leftFrontPower));
        leftRearPower   = Math.max(-1.0, Math.min(1.0, leftRearPower));
        rightFrontPower = Math.max(-1.0, Math.min(1.0, rightFrontPower));
        rightRearPower  = Math.max(-1.0, Math.min(1.0, rightRearPower));

        leftFrontPID.setPower(leftFrontPower);
        leftRearPID.setPower(leftRearPower);
        rightFrontPID.setPower(rightFrontPower);
        rightRearPID.setPower(rightRearPower);

        leftFrontPID.periodic();
        leftRearPID.periodic();
        rightFrontPID.periodic();
        rightRearPID.periodic();

        boolean positionReached = distance < 0.08;
        boolean headingReached  = Math.abs(headingError) < 0.05;

        if (positionReached && headingReached) {
            lastTargetX = Double.NaN;
            lastTargetY = Double.NaN;
            return true;
        }

        return false;
    }

    public boolean goToPosition(double targetX, double targetY, double targetHeading) {
        return goToPosition(targetX, targetY, targetHeading, 100);
    }

    public void resetPIDs() {
        leftFrontPID.reset();
        leftRearPID.reset();
        rightFrontPID.reset();
        rightRearPID.reset();
    }

    public void handleMCode(int code) {
        if (code == -1) return;

        switch (code) {
            case 1:
                lights.whiteOn();
                break;
            case 2:
                lights.redOn();
                break;
            case 3:
                lights.greenOn();
                break;
            case 4:
                lights.blueOn();
                break;
            default:
                break;
        }
    }

    public void stop() {
        leftFrontPID.stop();
        leftRearPID.stop();
        rightFrontPID.stop();
        rightRearPID.stop();
    }

    public static class PathConfig {
        private static final String aprilTagConfigPath = "/sdcard/FIRST/MephiFiles/Config.csv";

        public static Map<String, TagPose> loadAprilTagConfig() {
            Map<String, TagPose> tags = new HashMap<>();
            File file = new File(aprilTagConfigPath);
            if (!file.exists()) return tags;

            try (BufferedReader br = new BufferedReader(new FileReader(file))) {
                String line;
                boolean first = true;
                while ((line = br.readLine()) != null) {
                    if (first) { first = false; continue; }
                    String[] v = line.split(",");
                    if (v.length < 11) continue;
                    String key = v[1].trim() + "_" + v[2].trim();

                    tags.put(key, new TagPose(
                        Double.parseDouble(v[3]), Double.parseDouble(v[4]), Double.parseDouble(v[5]),
                        Double.parseDouble(v[6]), Double.parseDouble(v[7]), Double.parseDouble(v[8]),
                        Double.parseDouble(v[9]), Double.parseDouble(v[10])
                    ));
                }
            } catch (Exception e) { e.printStackTrace(); }
            return tags;
        }

        public static class PathPoint {
            public double x, y, heading;
            public int speedPct = 100;
            public int mCode = -1;

            public PathPoint(double x, double y, double heading) {
                this.x = x;
                this.y = y;
                this.heading = heading;
            }

            public PathPoint(double x, double y, double heading, int speedPct, int mCode) {
                this.x = x;
                this.y = y;
                this.heading = heading;
                this.speedPct = speedPct;
                this.mCode = mCode;
            }
        }

        public List<PathPoint> targets = new ArrayList<>();
        public double xOffset = 0.0;
        public double yOffset = 0.0;

        public PathConfig(String fileName) {
            File file = new File("/sdcard/FIRST/MephiFiles/AutoPaths/" + fileName);

            if (!file.exists()) return;

            try (BufferedReader br = new BufferedReader(new FileReader(file))) {
                String line;

                while ((line = br.readLine()) != null) {
                    line = line.trim();
                    if (line.isEmpty()) continue;

                    if (line.startsWith("; OFFSETS:")) {
                        String[] tokens = line.substring(10).trim().split("\\s+");
                        for (String token : tokens) {
                            if (token.startsWith("X=")) xOffset = Double.parseDouble(token.substring(2));
                            if (token.startsWith("Y=")) yOffset = Double.parseDouble(token.substring(2));
                        }
                        continue;
                    }

                    if (line.startsWith(";")) continue;

                    if (line.startsWith("G1")) {
                        String[] tokens = line.split("\\s+");
                        double x = 0.0, y = 0.0, heading = 0.0;
                        int speedPct = 100, mCode = -1;

                        for (String token : tokens) {
                            if (token.length() < 2) continue;
                            char cmd = Character.toUpperCase(token.charAt(0));
                            String val = token.substring(1);

                            switch (cmd) {
                                case 'X': x = Double.parseDouble(val) + xOffset; break;
                                case 'Y': y = Double.parseDouble(val) + yOffset; break;
                                case 'Z': heading = Math.toRadians(Double.parseDouble(val)); break;
                                case 'F': speedPct = Integer.parseInt(val); break;
                                case 'M': mCode = Integer.parseInt(val); break;
                            }
                        }

                        targets.add(new PathPoint(x, y, heading, speedPct, mCode));
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public static class MephistophelesPath {

        private final Mephistopheles mephi;
        private final Localization loc;
        private final PathConfig config;

        private int targetIndex = 0;

        public MephistophelesPath(Mephistopheles mephi, Localization loc, PathConfig config){
            this.mephi = mephi;
            this.loc = loc;
            this.config = config;
        }

        public boolean follow(){
            if(targetIndex >= config.targets.size()) return true;

            PathConfig.PathPoint target = config.targets.get(targetIndex);
            boolean reached = mephi.goToPosition(target.x, target.y, target.heading, target.speedPct);

            if(reached){
                mephi.handleMCode(target.mCode);
                targetIndex++;
            }
            return false;
        }
    }
}