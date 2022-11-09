/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for the robot.
 * <p>
 */
public class HuskyBot {
    /* Public OpMode members. */
    public DcMotorEx frontLeftDrive = null;
    public DcMotorEx frontRightDrive = null;
    public DcMotorEx rearLeftDrive = null;
    public DcMotorEx rearRightDrive = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    // add velocity constant: NEEDS TO BE UPDATED!
    public static final double VELOCITY_CONSTANT = 537.7 * 312/60;

    public HuskyBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotorEx.class, "front_left_drive");
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rear_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "front_right_drive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rear_right_drive");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);

        // this base configuration sets the drive motors to run without encoders and the arm motor
        // to run with encoder. if any opmode requires different setting, that should be changed in
        // the opmode itself
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

