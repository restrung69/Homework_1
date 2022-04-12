# Homework_1
Команды для запуска:
Запускается ядро ROS
- $ roscore


Экспортируем модель робота и запускаем окружающую среду:
- $ export TURTLEBOT3_MODEL=waffle
- $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Собираем только пакет beginner_tutorials:
- $ source devel/setup.bash
- $ catkin_make --only-pkg-with-deps beginner_tutorials

Запускаем файлы с кодом:
- $ rosrun robot_sim robot.py
- $ rosrun robot_sim server.py

Программа в файле server.py отвечает за планирование и расчет траектории движения робота (координаты, направление и т. п.), на основе получаемых от робота данных (robot.py).\
Программа в файле robot.py считывает показания с датчиков, и на их основе рассчитывает угловые скорости самого робота и его колес. Обменивается сообщениями с server.py\
Программа в файле listener.py не обязательна для запуска, она использовалась для вывода координат и направления робота
