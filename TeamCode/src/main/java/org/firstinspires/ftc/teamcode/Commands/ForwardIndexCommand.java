package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Util.Constants.indexerConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Indexer;

public class ForwardIndexCommand {
    private Indexer indexer;

    public ForwardIndexCommand(Indexer indexer) {
        this.indexer = indexer;
    }
    public void initialize() {}

    public void execute() {
        initialize();
        indexer.spinForward();
    }

     public void end() {
        indexer.stop();
    }
    
    public boolean isFinished() {
        return false;
    }
}