#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "geometry_msgs/Twist.h"

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Change robot|s moving state");
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x  = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service drive_bot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int       color_pixel;
    int       side;            // 0: centre, 1: left, 2: right, 3: out of sight
    long int  i;
    long int  left_limit;      // end of left part of image
    long int  right_limit;     // start of right part of image
    long int  end_of_frame;    // end of image
    long int  position;        // current pixel of image row
    bool      move;            // true: move robot, false: don't move

    left_limit   = 2 * img.step;     // 0.2*step
    right_limit  = 8 * img.step;     // 0.8*step
    end_of_frame = 10 * img.step;    // 1 * step (end of row)
    color_pixel  = 255;
    move         = false;

    side = 3;
    position = 0;

    // each pixel represented by 3 bytes: i: RED, i+1: BLUE, i+2: GREEN
    // first byte is red, with a step of 3 we read always the red byte.
    for (i=0; i<img.height*img.step; i+=3)
    {
        if((img.data[i]==color_pixel)&&(img.data[i+1]==color_pixel)&&(img.data[i+2]==color_pixel))
        {
            move = true;
            position = 10*(i%img.step);        // i+2 is the end of a pixel
            if((0<position)&&(position<=left_limit))
            {
                side = 1;
                drive_robot(0.1,0.5);          // move and turn left
            }
            else if((left_limit<position)&&(position<right_limit))
            {
                side = 0;
                drive_robot(0.1,0.0);         // move forward
            }
            else if(position<end_of_frame)
            {
                side = 2;
                drive_robot(0.1,-0.5);        // move and turn right
            }
            else
            {
                side = 3;
                drive_robot(0.0,0.0);         // don't move
            }
            break;
        }
    }

    if(move==false)   // no white pixel found
    {
        side = 3;
        drive_robot(0.0,0.0);
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
