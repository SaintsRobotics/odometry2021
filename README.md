# odometry2021
Repository for exploring Swerve Odometry

Odometry is constructed as m_odometry in the SwerveDrivetrain.java file and takes in a SwerveDriveKinematics object and Rotation2D object (initial heading)
The initial position is assumed to be the origin on the field. All calculations will be based on this initial position.
The position of the robot is updated each time the periodic function in SwerveDrivetrain is run
This update method takes in the state of each wheel and the current gyro angle to calculate the changing position of the robot
Currently, two values, the X and Y position of the robot, are being printed to SmartDashboard


Access to odometry occurs in SwerveDrivetrain.java
m_pose represents the last updated position of the robot. It is a Pose2D type.
To access other values given by odometry use https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/kinematics/SwerveDriveOdometry.html:
Optimally, code should go in the periodic() function

