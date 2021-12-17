# Road Runner Advanced Examples

You should be fairly familiar with Road Runner and the quickstart before taking a look at these samples.

The main quickstart repo can be found [here](https://github.com/acmerobotics/road-runner).

## Installation

For more detailed instructions on getting Road Runner setup in your own project, see the [Road Runner README](https://github.com/acmerobotics/road-runner#core).

1. Download or clone this repo with `git clone https://github.com/acmerobotics/road-runner-quickstart`.

1. Open the project in Android Studio and build `TeamCode` like any other `ftc_app` project.

## Samples:

1. Passing pose data between opmodes

    Files:
    - [teamcode/drive/advanced/TeleOpJustLocalizer.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java)
        
    - [teamcode/drive/advanced/AutoTransferPose.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AutoTransferPose.java)
        
    - [teamcode/drive/advanced/PoseStorage.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/PoseStorage.java)

    If you wish to read your localizer's pose during teleop, you need to know where your initial
    pose is. If an initial pose is not set, the program will assume you start at x: 0, y: 0, and
    heading: 0, which is most likely not what you want. So, to know where you are teleop, you must
    know where you ended in auto. This sample explains how to do so via a static class,
    `PoseStorage`, that allows data to persist between opmodes. AutoTransferPose will write its
    pose estimate to `PoseStorage.currentPose`. Because this is a static field, it will persist
    between opmodes. Thus, a teleop afterwards is able to read from it and know where the autonomous
    opmode ended.

2. Road Runner in teleop - just the localizer
    
    File:
    - [teamcode/drive/advanced/TeleOpJustLocalizer.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpJustLocalizer.java)

    Example code demonstrating how one would read from their localizer in teleop. Utilizes a static
    class to pass data between opmodes. This sample reads from the `PoseStorage` static field to set
    an initial starting pose. An autonomous opmode should have written its last known pose to
    the `PoseStorage.currentPose` field. 

3. Road Runner in teleop - incorporating the drive class
   
   File:
   - [teamcode/drive/advanced/TeleOpDrive.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpDrive.java)

    Example code demonstrating how one would incorporate the `SampleMecanumDrive` class into their
    teleop, without the need for a separate robot class. Instead, this sample utilizes the drive
    class's kinematics and `setDrivePower()` function. This sample is essentially just a modified
    `LocalizationTest.java` with pose extraction from `PoseStorage` and additional comments.
    
4. Road Runner in teleop - field centric drive
    
    File:
    - [teamcode/drive/advanced/TeleOpFieldCentric.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpFieldCentric.java)

     Example code demonstrating how one would implement field centric control using
     `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition
     of field centric control. To achieve field centric control, the only modification one needs is
     to rotate the input vector by the current heading before passing it into the inverse kinematics.

5. Road Runner in teleop - align to point

    File:
    - [teamcode/drive/advanced/TeleOpAlignWithPoint.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java)

    Example code demonstrating how to align the bot to a specified point in teleop. The user is able
    to switch from normal driver control mode to alignment mode. Alignment mode will switch into
    field centric and independently control heading via a PID controller to align with the specified
    point.

6. Async following with FSM orchestration

   File:
   - [teamcode/drive/advanced/AsyncFollowingFSM.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AsyncFollowingFSM.java)

    Example opmode describing how one utilizes async following in conjunction with finite state
    machines to orchestrate variable, multi-step movements. This allows for complex autonomous
    programs.

7. Breaking from a live trajectory

   Files:
   - [teamcode/drive/advanced/AutoBreakTrajectory.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/AutoBreakTrajectory.java)
   - [teamcode/drive/advanced/SampleMecanumDriveCancelable.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/SampleMecanumDriveCancelable.java)

    Example opmode describing how to break out from a live trajectory at any arbitrary point in time.
    This allows for neat things like incorporating live trajectory following in your teleop.
    `TeleOpAugmentedDriving.java` will build on this concept. This opmode will prematurely stop the
    following 3 seconds into starting it.
    
    This opmode utilizes the `SampleMecanumDriveCancelable.java` class. The default
    `SampleMecanumDrive.java` class does not allow for arbitrary following cancellation.

8. Automatic driving in teleop

   Files:
   - [teamcode/drive/advanced/TeleOpAugmentedDriving.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAugmentedDriving.java)
   - [teamcode/drive/advanced/SampleMecanumDriveCancelable.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/SampleMecanumDriveCancelable.java)

    **WARNING: THIS IS A DEMONSTRATION OF ROAD RUNNER'S CAPABILITIES. I DO NOT RECOMMEND DOING THIS IN GAME**

    Example opmode demonstrates how one can augment driver control by following arbitrary
    Road Runner trajectories at any time during teleop. This really isn't recommended at all. This
    is not what Trajectories are meant for. A path follower is more suited for this scenario. This
    sample primarily serves as a demo showcasing Road Runner's capabilities.
    
    See the comments for details on the driver augmentation.