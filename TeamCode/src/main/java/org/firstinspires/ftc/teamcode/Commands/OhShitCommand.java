import org.firstinspires.ftc.teamcode.Util.Constants.indexerConstants;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Indexer;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;

public class OhShitCommand {
    private Intake intake;
    private IntakeArm intakeArm;
    private Indexer indexer;
    private Shooter shooter;

    public OhShitCommand(Intake intake, IntakeArm intakeArm, Indexer indexer, Shooter shooter) {
        this.intake = intake;
        this.intakeArm = intakeArm;
        this.indexer = indexer;
        this.shooter = shooter;
    }
    public void initialize() {
        intakeArm.armDown();
    }

    public void execute() {
        initialize();
        intake.outtake();
        indexer.spinBackwards();
        shooter.spinBackwards();
    }

     public void end() {
        intake.stop();
        indexer.stop();
        shooter.stop();
    }
    
    public boolean isFinished() {
        return false;
    }
}