import org.firstinspires.ftc.teamcode.Util.Constants.indexerConstants;

public class BackwardIndexCommand {
    private Indexer indexer;

    public BackwardIndexCommand(Indexer indexer) {
        this.indexer = indexer;
    }
    public void initialize() {}

    public void execute() {
        initialize();
        indexer.spinBackward();
    }

     public void end() {
        indexer.stop();
    }
    
    public boolean isFinished() {
        return false;
    }
}