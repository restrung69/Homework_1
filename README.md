# Homework_1
Команды для запуска:
Запускается ядро ROS
- $ roscore
Экспортируем модель робота и запускаем окружающую среду
- $ export TURTLEBOT3_MODEL=waffle
- $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

Собираем только пакет beginner_tutorials
- $ source devel/setup.bash
- $ catkin_make --only-pkg-with-deps beginner_tutorials

Запускаем файлы с кодом
- $ rosrun robot_sim robot.py
- $ rosrun robot_sim server.py
