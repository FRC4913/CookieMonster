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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Auto", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyAuto extends HuskyAutoBase {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final double FORWARD_DISTANCE = 24;
        final double STRAFE_DISTANCE = 24;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(66)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(40)
                .build();

        // Start a separate thread to control the arm
        huskyBot.arm.run(this);

        waitForStart();
        runtime.reset();

        huskyBot.arm.clawLift.setPosition(1.0);

        huskyBot.arm.setPositionToGroundJunction();
        drive.followTrajectory(traj1);

        sleep(500);

        huskyBot.arm.clawGrab.setPosition(Arm.CLAW_GRAB_CLOSE_POSITION);

        sleep(500);

        huskyBot.arm.setPositionToMediumJunction();
        drive.followTrajectory(traj2);

        sleep(500);

        drive.turn(Math.toRadians(45));
        sleep(500);
        huskyBot.arm.clawGrab.setPosition(Arm.CLAW_GRAB_OPEN_POSITION);
        sleep(500);
        huskyBot.arm.setPositionToGroundJunction();
        sleep(1000);


        telemetry.update();
    }
}