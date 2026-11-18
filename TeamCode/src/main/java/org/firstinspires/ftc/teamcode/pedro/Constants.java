package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("LF");
                c.backLeftName.set("LB");
                c.frontRightName.set("RF");
                c.backRightName.set("RB");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.600379643477793);
        c.yPodOffset.set(0.08673809644744153);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2025446534262092);
                Controller secondaryTranslationalForward = Controller.proportional(0.0748348373481688);
                Controller primaryTranslationalLateral = Controller.proportional(0.26037050130206557);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09619994300318532);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018546573097690193));
                c.brake.set(Controller.proportionalFeedforward(0.015764587133036665));

                c.headingFeedback.set(Controller.proportional(3.6555524250107014));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04989619668328077, 0.004457844006906064));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0668260464404093, 0.04489060840002571));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012029484001212576, 0.0013996441694608195));

                c.maxAchievableForwardVelocity.set(49.430833529707726);
                c.maxAchievableStrafeVelocity.set(47.156622115193024);
                c.naturalForwardDeceleration.set(38.83434137122458);
                c.naturalStrafeDeceleration.set(55.349985827670984);
            }
    );
}