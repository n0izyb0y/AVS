#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <linux/input.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

#include "ros/ros.h"
#include "cmd_msgs/cmd.h"

#include <JHPWMPCA9685.h>


PCA9685 *pca9685 = new PCA9685();

float acceleration = 0.0;           // Переменная для величины ускорения.
float steering = 0.0;               // Переменная для угла поворота колёс.

float throttlePWMneutralPoint; // Величина ШИМ'а для контроллера двигателя в нейтральном положении педали газа/тормоза (acceleration = 0.0).
float steeringPWMneutralPoint; // Величина ШИМ'а для сервопривода рулевого управления в нейтральном положении руля (steering = 0.0).
float throttlePWMrate; // Величина изменения (от 0.0 до 1.0 и от -1.0 до 0.0) ШИМ'а для контроллера двигателя.
float steeringPWMrate; // Величина изменения (от 0.0 до 1.0 и от -1.0 до 0.0) ШИМ'а для сервопривода рулевого управления.
float throttleLimit;   // Ограничение максимального положения педали газа.


void masterInputCmdCallback(const cmd_msgs::cmd msg)
{

    acceleration = msg.acceleration;    // Получение команды ускорения из топика ROS'а.
    steering = msg.steering;            // Получение команды поворота колёс из топика ROS'а.

    if (acceleration > throttleLimit) acceleration = throttleLimit;   // Если положение педали газа больше ограничения, то положение = ограничению.
    //if (acceleration < -throttleLimit) acceleration = -throttleLimit; // Если положение педали газа меньше ограничения со знаком минус, то положение = ограничению со знаком минус.

    int steeringPWM = steeringPWMneutralPoint + round(steeringPWMrate * steering);   // Рассчёт величины ШИМ'а для контроллера двигателя.
    int accelerationPWM = throttlePWMneutralPoint + round(throttlePWMrate * acceleration);  // Рассчёт величины ШИМ'a для сервопривода рулевого управления.
    ROS_DEBUG_STREAM("[MoveController] - accPWM == " << accelerationPWM << " strPWM ==" << steeringPWM);
//    if (accelerationPWM == 340){
//       pca9685->setPWM(2,0,1);
//       usleep(50000);
//    }
//    else
    pca9685->setPWM(2,0, accelerationPWM);  // Отправка в I2C порт величины ШИМ'а для контроллера двигателя (2 канал).
    pca9685->setPWM(0,0, steeringPWM);      // Отправка в I2C порт величины ШИМ'а для сервопривода рулевого управления (0 канал).
}

int main (int argc, char *argv[])
{
    std::string inputTopicName;    // Имя топика для вывода управляющих команд.

    // Инициализация нода ROS
    ros::init(argc, argv, "move_controller");
    ros::NodeHandle nh("~");

    nh.param<std::string>("inputTopicName", inputTopicName, "/master_station/cmd");
    nh.param<float>("throttlePWMneutralPoint", throttlePWMneutralPoint, 340);
    nh.param<float>("steeringPWMneutralPoint", steeringPWMneutralPoint, 320);
    nh.param<float>("throttlePWMrate", throttlePWMrate, 140);
    nh.param<float>("steeringPWMrate", steeringPWMrate, 90);
    nh.param<float>("throttleLimit", throttleLimit, 1.0);

    ros::Subscriber commandSub = nh.subscribe<cmd_msgs::cmd>(inputTopicName, 1, masterInputCmdCallback);

    ROS_INFO_STREAM("[MoveController] - (START)");

    int err = pca9685->openPCA9685();
    if (err < 0)
    {
        ROS_ERROR_STREAM("[MasterStation] - (ERROR) - pca9685 error == " << pca9685->error);
    }
    else
    {
        ROS_DEBUG_STREAM("[MoveController] - PCA9685 Device Address == " << pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(50) ;
        pca9685->setPWM(0,0, 320);
        pca9685->setPWM(2,0, 1);
    }

    ros::spin();

    ROS_INFO_STREAM("[MoveController] - (FINISH)");

    return 0;
}
