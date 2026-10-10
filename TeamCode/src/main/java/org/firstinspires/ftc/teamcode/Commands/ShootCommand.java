package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.SubSystems.Mephistopheles;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.SubSystems.Localization;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class ShootCommand {
    private Mephistopheles mephi;
    private Vision vision;
    private Localization localization;
    private Shooter shooter;
    private int speedPct;
    
    private double[] targetPose = null;
    private boolean finished = false;
    private boolean shotPrepared = false;

    public ShootCommand(Mephistopheles mephi, Vision vision, Localization localization, Shooter shooter, int speedPct) {
        this.mephi = mephi;
        this.vision = vision;
        this.localization = localization;
        this.shooter = shooter;
        this.speedPct = speedPct;
    }

    public void initialize() {
        targetPose = vision.getActiveTargetPosition(localization);
        finished = false;
        shotPrepared = false;
    }

    public void execute() {
        if (targetPose == null) {
            initialize();
        }

        if (targetPose != null) {
            finished = mephi.goToPosition(targetPose[0], targetPose[1], targetPose[2], speedPct, "G01");
            if (finished && !shotPrepared) {
                shooter.prepareShot();
                shotPrepared = true;
            }
        } else {
            finished = true;
        }
    }
    
    public void end() {
        targetPose = null;
        finished = false;
        shotPrepared = false;
    }

    public boolean isFinished() {
        return finished;
    }
}