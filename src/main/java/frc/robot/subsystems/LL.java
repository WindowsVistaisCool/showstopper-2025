package frc.robot.subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;
import frc.thunder.vision.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class LL extends SubsystemBase {

    private final int pipeline = 0;
    private Limelight stopMe;
    private Thread poseProducer;
    private double lastVisionRead = 0;
    private Consumer<Pose4d> visionConsumer = (Pose4d _pose) -> {
    };

        private Field2d field = new Field2d();


    public LL() {
        stopMe = new Limelight("limelight-stopme", "10.8.62.11");

        stopMe.setPipeline(pipeline);

        setOrientations();

        this.poseProducer = new Thread(() -> {
            while (true) {
                try {
                    if (stopMe.getPipeline() == pipeline){
                        monitor(stopMe);
                    }
                    // monitor(champs);
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    System.err.println("Vision loop error: " + e.toString());
                    e.printStackTrace();
                }
            }
        });
        poseProducer.start();
    }

    private void monitor(Limelight limelight) {
        if (limelight.hasTarget()) {
            Pose4d pose = limelight.getBlueAlliancePose();
            if (pose.getFPGATimestamp() > lastVisionRead && pose.trust()) {
                visionConsumer.accept(pose);
                lastVisionRead = pose.getFPGATimestamp();
            }
        }
    }

    public void setApplyVisionUpdate(Consumer<Pose4d> visionConsumer) {
        this.visionConsumer = visionConsumer;
    }

    public Limelight getStopMe() {
        return stopMe;
    }
    public void setStopMePipeline(int pipeline) {
        stopMe.setPipeline(pipeline);
    }


    public int getStopMePipeline() {
        return stopMe.getPipeline();
    }

    @Override
    public void periodic() {
            field.setRobotPose(stopMe.getBlueAlliancePose().toPose2d());

            LightningShuffleboard.set("Vision", "LL_field", field);
    }



    public void setOrientations(){
        
    }

}
