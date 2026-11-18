package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

//gobuilda imports

import static com.pedropathing.api.Paths.line;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Example TeleOp")
public class CompTeleOp extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    public int LAUNCHER_TARGET_VELOCITY = 1250; //2678 RPM
    public int INTENDED_LAUNCHER_VELOCITY = 1250;
    public double INTENDED_LAUNCHER_POWER = 0.00;
    public double LAUNCHER_POWER = 0.00;
    private boolean shooting = false;

    private boolean VelocityPowerShift = false; //False = Velocity, True = Power

    public ElapsedTime timer;

    //public final int LAUNCHER_MIN_VELOCITY = 1200; //2571 RPM

    //Poses

    private final Pose leftEnd = poseFactory.of(60,118,270);
    //private final Pose rightStart = poseFactory.of(8.7, 8.7, 90);
    private final Pose rightEnd = poseFactory.of(60,22.7,90);


    double intakePower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);


        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop(){
        if(gamepad1.rightStickButtonWasPressed()) {
            Pose leftStart = poseFactory.of(8.7, 132, 270);
            follower.setPose(leftStart);
        }
        if(gamepad1.xWasPressed()){

            follower.follow(line(follower.pose(), leftEnd).linear(follower.pose(), leftEnd));
        } else if(gamepad1.bWasPressed()) {
            follower.follow(line(follower.pose(), rightEnd).linear(follower.pose(), rightEnd));
        }
        if(!follower.isBusy()) {
            follower.manual(-gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
            );
        }
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Launcher Target Velocity", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Launcher Target Power", LAUNCHER_POWER);
        telemetry.addData("Is Power Mode On: ", VelocityPowerShift);


        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;


        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        if(!VelocityPowerShift) {
            if (gamepad1.dpad_up) {
                INTENDED_LAUNCHER_VELOCITY += 5;
            } else if (gamepad1.dpad_down) {
                INTENDED_LAUNCHER_VELOCITY -= 5;
            }
        } else {
            if (gamepad1.dpad_up) {
                INTENDED_LAUNCHER_POWER += 0.01;
            } else if (gamepad1.dpad_down) {
                INTENDED_LAUNCHER_POWER -= 0.01;
            } else if (gamepad1.dpad_right) {
                INTENDED_LAUNCHER_POWER += 0.005;
            } else if (gamepad1.dpad_left) {
                INTENDED_LAUNCHER_POWER -= 0.005;
            }
        }

        if(INTENDED_LAUNCHER_POWER > 1) {
            INTENDED_LAUNCHER_POWER = 1;
        } else if (INTENDED_LAUNCHER_POWER < 0) {
            INTENDED_LAUNCHER_POWER = 0;
        } else {
            LAUNCHER_POWER = INTENDED_LAUNCHER_POWER;
        }

        if(INTENDED_LAUNCHER_VELOCITY> 2250) {
            INTENDED_LAUNCHER_VELOCITY = 1250;
        } else if (INTENDED_LAUNCHER_VELOCITY < 200) {
            INTENDED_LAUNCHER_VELOCITY = 200;
        } else {
            LAUNCHER_TARGET_VELOCITY = INTENDED_LAUNCHER_VELOCITY;
        }
        if (!VelocityPowerShift) {
            if (gamepad1.right_bumper) {
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else {
                launcher.setVelocity(0);
            }
        } else {
            if (gamepad1.right_bumper) {
                launcher.setPower(LAUNCHER_POWER);
            } else {
                launcher.setPower(0);
            }
        }
        if(gamepad1.aWasPressed()) {
            VelocityPowerShift = !VelocityPowerShift;
        }

        if(gamepad1.yWasPressed()) {
            timer = new ElapsedTime();
            timer.reset();
            shooting = true;
        }
        if (shooting) {
            while (timer.seconds() < 3.8) {
                launcher.setPower(0.6);
                if (timer.seconds() > 2.5 && timer.seconds() < 3.5) {
                    windmillServo.setPower(-0.5);
                }
            }
            shooting = false;
        }
        if (gamepad1.left_bumper) {
            windmillServo.setPower(-1);
        } else {
            windmillServo.setPower(0);
        }

    }
    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }
}
