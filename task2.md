Тема: Навигация TurtleBot 3 в Gazebo: SLAM и построение карты.
Цель: изучить основы навигации робота в неизвестном окружении, освоить технологию SLAM (Simultaneous Localization and Mapping) для построения карты окружения и научиться планировать маршрут в ROS 2.

1. Подготовка и установка
Требования:
    • Ubuntu 20.04 LTS или 22.04 LTS;
    • ROS 2 Humble Hawksbill;
    • Gazebo 11;
    • пакеты TurtleBot 3;
    • пакеты для SLAM и навигации: slam_toolbox, nav2.
Установка дополнительных пакетов:

sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

2. Запуск симуляции с поддержкой SLAM
    1. Откройте терминал и запустите ROS 2 Master:

ros2 daemon start

    2. Запустите Gazebo с TurtleBot 3 и включением SLAM:

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    3. В новом терминале запустите SLAM‑узел:

ros2 launch slam_toolbox online_async_launch.py

    4. Проверьте, что SLAM запущен и принимает данные от лидара:

ros2 topic echo /scan

3. Построение карты окружения
Запустите RViz 2 для визуализации:

ros2 run rviz2 rviz2
    2. Настройте RViz:
    • добавьте отображение карты (Add → Map, топик /map);
    • добавьте отображение робота (Add → RobotModel);
    • добавьте отображение лидара (Add → LaserScan, топик /scan).
    3. Управляйте роботом вручную через teleop:

ros2 run turtlebot3_teleop teleop_keyboard

Перемещайте робота по сцене, наблюдая в RViz, как строится карта.

4. Сохраните построенную карту:

ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: map1"

4. Навигация по карте
    1. Остановите SLAM‑узел (Ctrl+C в терминале).
    2. Запустите навигацию:

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

    3. В RViz:
    • установите позицию робота через 2D Pose Estimate (клик правой кнопкой мыши);
    • задайте цель движения через 2D Goal Pose (клик левой кнопкой мыши).
    4. Наблюдайте, как робот планирует маршрут и движется к цели.

5. Программирование навигации
Создайте Python‑скрипт для автоматического движения к заданной точке:
    1. Создайте пакет:
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python tb3_navigation_demo

    2. В папке tb3_navigation_demo/tb3_navigation_demo/ создайте файл nav_to_pose.py:
#!/! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigationDemo(Node):
    def __init__(self):
        super().__init__('navigation_demo')
        self.navigator = BasicNavigator()

    def go_to_point(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = theta
        goal_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(goal_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isNavComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal reached!')
        else:
            self.get_logger().error('Navigation failed!')

def main():
    rclpy.init()
    nav_demo = NavigationDemo()
    nav_demo.go_to_point(2.0, 1.0, 0.0)
    rclpy.spin(nav_demo)
    nav_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    3. Соберите пакет:

cd ~/ros2_ws
colcon build
source install/setup.bash

    4. Запустите скрипт:

ros2 run tb3_navigation_demo nav_to_pose.py

6. Задания для самостоятельной работы
Модифицируйте скрипт навигации, чтобы робот объезжал препятствия, используя данные лидара.
Создайте миссию из нескольких точек назначения и реализуйте последовательное движение робота между ними.
Экспериментируйте с параметрами SLAM (например, resolution, max_range) и оцените их влияние на качество карты.
Добавьте визуализацию траектории движения робота в RViz.
Реализуйте алгоритм поиска пути (например, A* или Dijkstra) и сравните его с планировщиком Nav2.
7. Контрольные вопросы:
Что такое SLAM? Какие сенсоры обычно используются для SLAM?
Какие основные этапы навигации робота вы использовали в работе?
Что такое «карта занятости» (occupancy grid)? Как она используется в навигации?
Какие топики и сервисы используются для управления навигацией в ROS 2?
В чём разница между SLAM и автономной навигацией?
Как задать начальную позицию робота в RViz? Почему это важно?
Какие параметры можно настроить в SLAM для улучшения качества карты?
8. Отчёт по лабораторной работе:
Отчёт должен содержать:
    1. титульный лист с названием работы, ФИО студента, группой и датой;
    2. цель работы;
    3. описание выполненных шагов (с командами и скриншотами RViz на этапах SLAM и навигации);
    4. код скрипта навигации с комментариями;
    5. результаты выполнения заданий для самостоятельной работы (код и скриншоты);
    6. ответы на контрольные вопросы;
    7. выводы (что нового узнали, с какими трудностями столкнулись, как их преодолели, какие параметры SLAM оказались наиболее важными).