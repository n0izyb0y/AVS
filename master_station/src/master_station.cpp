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

// Коды клавиш клавиатуры ноутбука
#define UP_ARROW_KEY 	103
#define DOWN_ARROW_KEY 	108
#define LEFT_ARROW_KEY 	105
#define RIGHT_ARROW_KEY 106

// Кода клавиш bluetooth клавиатуры logitech
//#define UP_ARROW_KEY 	106
//#define DOWN_ARROW_KEY 	105
//#define LEFT_ARROW_KEY 	108
//#define RIGHT_ARROW_KEY 103

bool UpArrowPressed, DownArrowPressed, LeftArrowPressed, RightArrowPressed; // Флаги нажатия клавиш.

std::string deviceInputAdress;  // Путь до файла с событиями нужного устройства.
std::string outputTopicName;    // Имя топика для вывода управляющих команд.
float incThrottleRate;          // Величина УВЕЛИЧЕНИЯ положения педали газа при НАЖАТОЙ стрелке вверх (от 0.00 до 1.00).
float incBrakeRate;             // Величина увеличения положения педали тормоза при НАЖАТОЙ стрелке вниз (от 0.00 до -1.00).
float incSteeringRate;          // Величина увеличения угла поворота руля при нажатой НАЖАТОЙ влево/вправо (от 0.00 до 1.00).
float decThrottleRate;          // Величина УМЕНЬШЕНИЯ положения педали газа при ОТПУЩЕННОЙ стрелке вверх (от 1.00 до 0.00).
float decBrakeRate;             // Величина УМЕНЬШЕНИЯ положения педали тормоза при ОТПУЩЕННОЙ стрелке вниз (от -1.00 до 0.00).
float decSteeringRate;          // Величина УМЕНЬШЕНИЯ угла поворота руля при ОТПУЩЕННОЙ стрелке влево/вправо (от -1.00/1.00 до 0.00)

int main (int argc, char *argv[])
{
    char deviceName[256] = "Unknown"; // Переменная для имени устройства, события которого будут считываться.
    float acceleration = 0.0; // Команда для ускорения (положение педали газа/торомоза).
    float steering = 0.0; // Команда для угла поворота колёс (положение руля).

    struct input_event inputEvent[64]; // Переменная для чтения событий клавиатуры. Почему 64 - неизвестно, но работает.
    int fd, rd, size = sizeof (struct input_event); // Дескрипторы на открытие и чтение буфера событий клавиатуры и размер.

    // Инициализация нода ROS
    ros::init(argc, argv, "master_station");
    ros::NodeHandle nh("~");

    // Считывание параметров из yaml файла
    nh.param<std::string>("deviceInputAdress", deviceInputAdress, "/dev/input/event4");
    nh.param<std::string>("outputTopicName", outputTopicName, "cmd");

    nh.param<float>("incThrottleRate", incThrottleRate, 0.025);
    nh.param<float>("incBrakeRate", incBrakeRate, 0.025);
    nh.param<float>("incSteeringRate", incSteeringRate, 1.0);
    nh.param<float>("decThrottleRate", decThrottleRate, 0.05);
    nh.param<float>("decBrakeRate", decBrakeRate, 0.05);
    nh.param<float>("decSteeringRate", decSteeringRate, 1.0);

    // Перевод string в *char чтобы функция open и read для чтения буфера событий работали корректно.
    const char *charDeviceInputAdress = deviceInputAdress.c_str();

    ros::Publisher commandPub = nh.advertise<cmd_msgs::cmd>(outputTopicName, 1);

    ros::Rate loop_rate(200);

    //// ПЕРЕВОД ТЕРМИНАЛА В RAW РЕЖИМ ////
    // Структура для хранения старых настроек терминала (до изменений).
    struct termios orig_term_attr;
    // Структура для хранения новых настроек терминала (после изменений).
    struct termios new_term_attr;
    // Сохранение старых настроек терминала.
    tcgetattr(fileno(stdin), &orig_term_attr);
    // Копирование старых настроек в новые
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    // Изменение настроек терминала. Отключен вывод символов в консоль.
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    // Применение новых настроек для терминала.
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    //// РАЗЛИЧНЫЕ ПРОВЕРКИ /////
    // Проверка запуска программы под root'ом.
    if ((getuid ()) != 0)
    {
        ROS_ERROR_STREAM("[MasterStation] - (ERROR) - Not a root user");
        // Возврат старых настроек для терминала
        tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);
        return -1;
    }
    // Открытие файла с событиями устройства (клавиатуры) и проверка успешности этой операции.
    // O_NONBLOCK нужен, чтобы цикл крутился дальше, не ожидая нового события клавиатуры.
    if ((fd = open(charDeviceInputAdress, O_RDONLY | O_NONBLOCK)) == -1)
    {
        ROS_ERROR_STREAM("[MasterStation] - (ERROR) - Can not get acess to " << charDeviceInputAdress);
        // Возврат старых настроек для терминала
        tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);
        return -1;
    }

    // Доступ к буферу событий клавиатуры через дескриптор.
    ioctl (fd, EVIOCGNAME (sizeof(deviceName)), deviceName);

    ROS_INFO_STREAM("[MasterStation] - (START)");
    ROS_INFO_STREAM("[MasterStation] - Press ESC to Finish");

    //// ОСНОВНОЙ ЦИКЛ СЧИТЫВАНИЯ СОБЫТИЙ КЛАВИАТУРЫ ////
    while (ros::ok)
    {
        cmd_msgs::cmd command; // Переменная с командами положения педали газа/тормоза, руля и состояния для публикации в топик.

        rd = read(fd, inputEvent, size * 64); // Чтение потока событий клавиатуры. Почему размер * 64 - неизветсно.

        // Определяем, какая кнопка была нажата
        if (inputEvent[1].value == 1 && inputEvent[1].type == 1)
        {
            switch (inputEvent[1].code)
            {
            case UP_ARROW_KEY:
                UpArrowPressed = true;
                break;
            case DOWN_ARROW_KEY:
                DownArrowPressed = true;
                break;
            case LEFT_ARROW_KEY:
                LeftArrowPressed = true;
                break;
            case RIGHT_ARROW_KEY:
                RightArrowPressed = true;
                break;
            }

            // Если была нажата клавиша ESC - выходим из цикла.
            if (inputEvent[1].code == 1)
                break;
                //break;
        }
        // Определяем, какая кнопка была отпущена
        if (inputEvent[1].value == 0 && inputEvent[1].type == 1)
        {
            switch (inputEvent[1].code)
            {
            case UP_ARROW_KEY:
                UpArrowPressed = false;
                break;
            case DOWN_ARROW_KEY:
                DownArrowPressed = false;
                break;
            case LEFT_ARROW_KEY:
                LeftArrowPressed = false;
                break;
            case RIGHT_ARROW_KEY:
                 RightArrowPressed = false;
                break;
            }
        }
        // Плавное увеличение газа при нажатии стрелки вперёд.
        if (UpArrowPressed)
            (acceleration >= 1.0) ? acceleration = 1.0 : acceleration += incThrottleRate;
        // Плавное увеличение тормоза/заднего хода при нажатии стрелки назад.
        if (DownArrowPressed)
            (acceleration <= -1.0) ? acceleration = -1.0 : acceleration -= incBrakeRate;
        // Плавное увеличение поворота руля при нажатии стрелки вправо.
        if (RightArrowPressed)
            (steering >= 1.0) ? steering = 1.0 : steering += incSteeringRate;
        // Плавное увеличение поворота руля при нажатии стрелки влево.
        if (LeftArrowPressed)
            (steering <= -1.0) ? steering = -1.0 : steering -= incSteeringRate;

        // Плавное уменьшение газа/тормоза при отпускании стрелок вперёд/назад.
        if ((!UpArrowPressed && !DownArrowPressed) || (UpArrowPressed && DownArrowPressed))
        {
            if (acceleration >= decThrottleRate)
                acceleration -= decThrottleRate;
            if (acceleration <= -decBrakeRate)
                acceleration += decBrakeRate;
            if (acceleration < decThrottleRate && acceleration > -decBrakeRate)
                acceleration = 0.0;
        }

        // Плавное уменьшение угла поворота колёс при отпускании стрелок влево/вправо.
        if ((!RightArrowPressed && !LeftArrowPressed) || (RightArrowPressed && LeftArrowPressed))
        {
            if (steering >= decSteeringRate)
                steering -= decSteeringRate;
            if (steering <= -decSteeringRate)
                steering += decSteeringRate;
            if (steering < decSteeringRate && steering > -decSteeringRate)
                steering = 0.0;
        }

        // Округление float до сотых, чтобы не было мусора.
        command.acceleration = floor(acceleration * 100) / 100;
        command.steering = floor(steering * 100) / 100;

        commandPub.publish(command);

        ros::spinOnce();
        loop_rate.sleep();
    }
    // Закрытия файлового дескриптора буфера событий клавиатуры
    close(fd);

    // Возврат старых настроек для терминала
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    ROS_INFO_STREAM("[MasterStation] - (FINISH)");

    return 0;
}
