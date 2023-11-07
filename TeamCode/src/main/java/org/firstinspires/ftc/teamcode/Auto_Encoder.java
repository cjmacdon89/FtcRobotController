package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name="Auto_Encoder")
public class Auto_Encoder extends LinearOpMode {

    ElapsedTime timmer = new ElapsedTime();
    DcMotor LB_Drive;
    DcMotor RB_Drive;
    DcMotor LF_Drive;
    DcMotor RF_Drive;
    double power = 0.3;
    double park = 0;
    double distanceSensor = 500;
    static final double FEET_PER_METER = 3.28084;
    double current_position = 0;

    static final int run_time = 1000;
    double Forward_Encoder = 0;
    double Side_Encoder = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Motor Mapping
        LB_Drive = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        timmer.reset();
        while(timmer.milliseconds() < run_time){
            LB_Drive.setPower(-power);
            RB_Drive.setPower(-power);
            LF_Drive.setPower(-power);
            RF_Drive.setPower(-power);

            Forward_Encoder = LF_Drive.getCurrentPosition();
            Side_Encoder = RF_Drive.getCurrentPosition();

            telemetry.addData("Forward_Encoder", Forward_Encoder);
            telemetry.addData("Side_Encoder", Side_Encoder);
            telemetry.update();
        }

    }

    }