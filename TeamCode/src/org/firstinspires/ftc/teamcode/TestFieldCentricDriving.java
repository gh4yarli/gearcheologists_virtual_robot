package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp
public class TestFieldCentricDriving extends LinearOpMode {

    GoBildaPinpointDriver odo;
    DcMotorEx bl, fl, fr, br;

    public void runOpMode(){

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.setOffsets(100, 100, DistanceUnit.MM);
        odo.setEncoderResolution(336.88, DistanceUnit.INCH);

        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fr = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        //reverse direction
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);


        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);


        Pose2D pinPose = odo.getPosition();
        double velXInch = odo.getVelX(DistanceUnit.INCH);
        double velYInch = odo.getVelY(DistanceUnit.INCH);
        double headingVelDegrees = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        telemetry.addData("Pinpoint Offsets", "xOff: %.1f  yOff: %.1f",
                odo.getXOffset(DistanceUnit.INCH), odo.getYOffset(DistanceUnit.INCH));

        telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", odo.getEncoderX(), odo.getEncoderY());
        telemetry.update();


        telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", velXInch, velYInch, headingVelDegrees);

        telemetry.addData("Device Scalar :", odo.getYawScalar());


        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                odo.resetPosAndIMU();
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            }

            odo.update();

            pinPose = odo.getPosition();
            double pinX = odo.getPosX(DistanceUnit.INCH);
            double pinY = odo.getPosY(DistanceUnit.INCH);
            double pinH = odo.getHeading(AngleUnit.DEGREES);
            double pinVX = odo.getVelX(DistanceUnit.INCH);
            double pinVY = odo.getVelY(DistanceUnit.INCH);
            double pinVH = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinPose.getX(DistanceUnit.INCH),
                    pinPose.getY(DistanceUnit.INCH), pinPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint Pos", "x: %.1f  y: %.1f  h: %.1f", pinX, pinY, pinH);
            telemetry.addData("Pinpoint Vel", "x: %.1f  y: %.1f  h: %.1f", pinVX, pinVY, pinVH);
            telemetry.addData("Pinpoint Encoders", "X: %d  Y: %d", odo.getEncoderX(), odo.getEncoderY());

            telemetry.update();

            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double pBL = forward - strafe - rotate; //BackLeft
            double pFL = forward + strafe - rotate; //ForwardLeft
            double pFR = forward - strafe + rotate; //ForwardRight
            double pBR = forward + strafe + rotate; //Back Right

            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            double cosAngle = Math.cos((Math.PI / 2) - heading);
            double sinAngle = Math.sin((Math.PI / 2) - heading);

            double globalForward = -forward * sinAngle + strafe * cosAngle;
            double globalStrafe = forward * cosAngle + strafe * sinAngle;

            double globalPfl = globalForward + globalStrafe + rotate; //ForwardLeft
            double globalPfr = globalForward - globalStrafe - rotate; //ForwardRight
            double globalPbl = globalForward - globalStrafe + rotate; //BackLeft
            double globalPbr = globalForward + globalStrafe -
                    +rotate; //Back Right

            fl.setPower(globalPfl); // Forward Left
            fr.setPower(globalPfr); // Forward Right
            bl.setPower(globalPbl); // Back Left
            br.setPower(globalPbr); // Back Right

            /*double max = Math.max(1, Math.max(Math.abs(forward), Math.max(Math.abs(strafe), Math.abs(rotate))));
            if (max > 1){
                pBL /= max;
                pFL /= max;
                pFR /= max;
                pBR /= max;
            }*/


        }

    }

}
