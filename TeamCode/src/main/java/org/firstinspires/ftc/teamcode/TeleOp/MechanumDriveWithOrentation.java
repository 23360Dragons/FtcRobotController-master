package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class MechanumDriveWithOrentation extends OpMode {
    IMU _orient;
    DcMotor _intake;
    DcMotor _rightFront;
    DcMotor _leftFront;
    DcMotor _rightBack;
    DcMotor _leftBack;
    DcMotor _linear1;
    DcMotor _linear2;
    Servo _airplane;
    private double servoPosition;
    private org.firstinspires.ftc.robotcore.external.navigation.AngleUnit AngleUnit;
    Servo claw1;
    Servo claw2;
    final static double Claw_Home = 0.0;

    Servo arm1;
    Servo arm2;

    private double pos1;
    private double pos2;
    @Override
    public void init() {
        arm1 = hardwareMap.get(Servo.class, "Arm1");
        arm2 = hardwareMap.get(Servo.class, "Arm2");
        arm1.setDirection(Servo.Direction.REVERSE);
        //Starting position IS 0.65, up position is 0.45
        pos1 = 0.65;
        pos2 = 0.65;
        arm2.setPosition(pos2);
        arm1.setPosition(pos1);
        _linear1 = hardwareMap.get(DcMotor.class, "Linear1");
        _linear2 = hardwareMap.get(DcMotor.class, "Linear2");

        _linear2.setDirection(DcMotor.Direction.REVERSE);

        _linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Resets encoders to zero
        _linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Turns encoders back on
        _linear1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _linear2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _intake = hardwareMap.get(DcMotor.class, "Intake");
        _orient = hardwareMap.get(IMU.class, "BHI260AP");
        _leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        _rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        _leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        _rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        //assumption - right side reversed
        _rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        _airplane = hardwareMap.get(Servo.class, "Airplane");
        servoPosition = 1;
        _airplane.setPosition(servoPosition);
        claw1 = hardwareMap.get(Servo.class, "Claw1");
        claw2 = hardwareMap.get(Servo.class, "Claw2");
        claw1.setDirection(Servo.Direction.REVERSE);

        claw1.setPosition(Claw_Home);
        claw2.setPosition(Claw_Home);
       /* AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
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
    }

    @Override
    public void loop() {
// variables make the math easier to type, these have to be in the loop or it won't work
        int position1 = _linear1.getCurrentPosition();
        int position2 = _linear2.getCurrentPosition();
        double power = -gamepad2.left_stick_y;
        double y;
        double x;
        double rightX;
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        //we have to initialize the variable to control speed percentage
        double speed = -0.5;


        // If a button is held it can change the speed percentage
        if (gamepad1.x) {
            speed = -0.5;
        } else if (gamepad1.a) {
            speed= -0.25;
        } else if (gamepad1.b) {
            speed = -0.75;
        } else if (gamepad1.y) {
            speed = -1.0;
        }


    //This makes the mechanum works properly with math
        _leftFront.setPower(((y + x) + rightX)*speed);
        _leftBack.setPower(((y - x) + rightX)*speed);
        _rightFront.setPower(((y - x) - rightX)*speed);
        _rightBack.setPower(((y + x) - rightX)*speed);

        //Auto orientation program
        YawPitchRollAngles robotOrientation;
        robotOrientation = _orient.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        telemetry.addData("yaw: ", yaw);
        telemetry.update();


        if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right) {
            if (gamepad1.dpad_up && yaw != 0) {
                _leftFront.setPower(-0.05 * yaw);
                _leftBack.setPower(-0.05 * yaw);
                _rightFront.setPower(0.05 * yaw);
                _rightBack.setPower(0.05 * yaw);
            }
            if (gamepad1.dpad_down && yaw != -180) {
                _leftFront.setPower(0.05 * yaw);
                _leftBack.setPower(0.05 * yaw);
                _rightFront.setPower(-0.05 * yaw);
                _rightBack.setPower(-0.05 * yaw);
            }
            if (gamepad1.dpad_left && yaw != 90) {
                _leftFront.setPower(0.05 * yaw);
                _leftBack.setPower(0.05 * yaw);
                _rightFront.setPower(-0.05 * yaw);
                _rightBack.setPower(-0.05 * yaw);
            }
            if (gamepad1.dpad_right && yaw != -90) {
                _leftFront.setPower(-0.05 * yaw);
                _leftBack.setPower(-0.05 * yaw);
                _rightFront.setPower(0.05 * yaw);
                _rightBack.setPower(0.05 * yaw);
            }
        } else {

            _leftFront.setPower(((y + x) + rightX) * speed);
            _leftBack.setPower(((y - x) + rightX) * speed);
            _rightFront.setPower(((y - x) - rightX) * speed);
            _rightBack.setPower(((y + x) - rightX) * speed);
        }
        //add comment about which way the intake runs
        //is negative in or out?
        if (gamepad2.b)
        {
            _intake.setPower(-0.75) ;
        }
        else if (gamepad2.x)
        {
            _intake.setPower(0.75);
        } else
        {
            _intake.setPower(0);
        }


        if (power > 0 /*|| position1 < 4355 || position2 < 4355*/) {
            _linear1.setPower(0.5);
            _linear2.setPower(0.5);
        } else if (power < 0 /*|| position1 >= 0 || position2 >= 0*/) {
            _linear1.setPower(-0.5);
            _linear2.setPower(-0.5);
        } else {
            _linear1.setPower(0);
            _linear2.setPower(0);
        }
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            servoPosition = 0;
            _airplane.setPosition(servoPosition);
        }

        if (gamepad2.a) {
            claw1.setPosition(0);
            claw2.setPosition(0);
        }else if (gamepad2.y) {
            claw1.setPosition(0.35);
            claw2.setPosition(0.35);
        }
        if (gamepad2.dpad_left){
            pos2 = 0.4;
            pos1 = 0.4;
            arm1.setPosition(pos1);
            arm2.setPosition(pos2);
        } else if (gamepad2.dpad_right) {
            pos2 = 0.65;
            pos1 = 0.65;
            arm1.setPosition(pos1);
            arm2.setPosition(pos2);
        }
    }
    }