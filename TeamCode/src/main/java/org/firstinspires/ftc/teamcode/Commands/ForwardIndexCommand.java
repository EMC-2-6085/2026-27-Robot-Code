import org.firstinspires.ftc.teamcode.Util.Constants.indexerConstants;

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