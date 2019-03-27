#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call DriveToTarget");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int ball_x = -1, ball_y = -1, white_pixel_count = 0;
    float white_pixel_cutoff = 0.70;
    int channels = img.step / img.width;

    // Loop through each pixel looking for a bright white (r,b,g=255) one.
    // Keep track of the number of pixels that fit this criteria.
    // Assume RGB or BGR aligned at the beginning of each step.
    for (int i = 0; i < img.height * img.step; i += channels)
    {
        if (img.data[i] == white_pixel &&
            img.data[i + 1] == white_pixel &&
            img.data[i + 2] == white_pixel)
        {
            if (ball_x == -1) {
                ball_y = i / img.step;
                ball_x = (i % img.step) / channels;
            }
            white_pixel_count += 1;
        }
    }

    // ENHNACEMENT: Calculate the % of pixels that are white. This is used to stop the
    // robot when the white ball is touching the nose.
    float white_pixel_percent = (float)white_pixel_count / (img.height * img.width);
    ROS_INFO_STREAM("White pixel percent: " + std::to_string(white_pixel_percent));

    // Ball X,Y pixels are aligned with the far left of the camera (i.e. the first pixel in the image).
    if (ball_x != -1 && ball_y != -1 && white_pixel_percent < 0.70)
    {
        float third = img.width / 3;
        float ang_z;
        if (ball_x < third)
        {
            // Pixel is in the left third, so go left at an appropriate angle.
            ang_z = 0.5 * (third / (ball_x + 1));
            ROS_INFO_STREAM("Going left: " + std::to_string(ang_z));
            drive_robot(0.5, ang_z);
        }
        else if (ball_x < third * 2)
        {
            // Pixel is in the middle third, so go straight.
            ROS_INFO_STREAM("Going straight");
            drive_robot(0.5, 0);
        }
        else
        {
            // Pixel is in the right third, so go right at an appropriate angle.
            ang_z = -0.5 * ball_x / img.width;
            ROS_INFO_STREAM("Going right: " + std::to_string(ang_z));
            drive_robot(0.5, ang_z);
        }
    }
    else
    {
        drive_robot(0, 0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
