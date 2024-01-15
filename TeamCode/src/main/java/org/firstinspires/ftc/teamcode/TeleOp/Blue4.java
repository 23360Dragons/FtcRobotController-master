package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Autonomous
public class Blue4 extends LinearOpMode {

    IMU _orient;
    DcMotor _intake;
    DcMotor _rightFront;
    DcMotor _leftFront;
    DcMotor _rightBack;
    DcMotor _leftBack;

    @Override
    public void runOpMode() throws InterruptedException {

        /*AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcamera"))
                .setCameraResolution(new Size(640,480))
                .build();*/

        _intake = hardwareMap.get(DcMotor.class, "Intake");
        _orient = hardwareMap.get(IMU.class, "BHI260AP");
        _leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        //assumption - right side reversed
        _rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
       // _intakeRamp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //For encoders
        int lF = _leftFront.getCurrentPosition();
        int rF = _rightFront.getCurrentPosition();
        int lB = _leftBack.getCurrentPosition();
        int rB = _rightBack.getCurrentPosition();

        YawPitchRollAngles robotOrientation;
        robotOrientation = _orient.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);


        waitForStart();
        _leftFront.setPower(0.75);
        _leftBack.setPower(-0.75);
        _rightFront.setPower(-0.75);
        _rightBack.setPower(0.75);
        Thread.sleep(1500);
        _leftFront.setPower(0);
        _leftBack.setPower(0);
        _rightFront.setPower(0);
        _rightBack.setPower(0);

    }
}

