#include <ros/ros.h>
#include <bitset>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

#include <unistd.h>                     //Needed for I2C port
#include <fcntl.h>			//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port

int file_i2c;
ros::Time last_msg;

template <typename T> std::string binary_repr(T value) {
       // The bitset size is set to match the type size.
       return std::bitset<sizeof(T)*8>(value).to_string();
    }

void setVelocity( float vRight, float vLeft)
{
	float buffer[2];
	buffer[0] = vRight;
	buffer[1] = vLeft;

	char *s = (char *) &buffer;
	write(file_i2c, s, 2*sizeof(float));
}

void JoyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_msg = ros::Time::now();

    float vright = msg->linear.x + 0.14*msg->angular.z; //*copysignf(1,msg->linear.x);
    float vleft = msg->linear.x - 0.14*msg->angular.z; //*copysignf(1,msg->linear.x);

    setVelocity(vright, vleft);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MagCar_wheels");

    ros::NodeHandle n;

    char *filename = (char*)"/dev/i2c-1";

    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        printf("Failed to open the i2c bus");
        return 0;
    }

    int addr = 0x12;

    if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return 0;
    }

    ros::Subscriber sub = n.subscribe("/cmd_vel", 2, JoyCallback);

    ros::Rate r(100);

    double i = 0;
    while (ros::ok())
    {
	ros::spinOnce();
    	if((ros::Time::now() - last_msg).toSec() > 0.1)
		setVelocity(0,0);
        r.sleep();
   }
  return 0;
}
