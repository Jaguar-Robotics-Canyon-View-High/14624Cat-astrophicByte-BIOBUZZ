package org.firstinspires.ftc.teamcode.autos;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class ExampleAuto extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(72, 12, 90);
    private final Pose middle1Pose = poseFactory.of(72, 72, 180);
    private final Pose middle2Pose = poseFactory.of(24, 72, 180);
    private final Pose parkPose = poseFactory.of(24, 24, 270);

    // Path methods
    private Path startToMiddle1() {
        return line(startPose, middle1Pose).linear(startPose, middle1Pose);
    }

    private Path middle1ToMiddle2(){
        return line(middle1Pose, middle2Pose).linear(middle1Pose, middle2Pose);
    }

    private Path middle2toPark(){
        return line(middle2Pose, parkPose).linear(middle2Pose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToMiddle1()),
                // Add mechanism commands here.
                follow(follower, middle1ToMiddle2()),
                follow(follower, middle2toPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}