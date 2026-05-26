# Отчёт по лабораторной работе №1
## Тема: Введение в ROS 2 и симуляцию TurtleBot 3 в Gazebo

Генералов Даниил Михайлович, НПИмд-01-26

**Рабочая папка для этой лабораторной работы опубликована по адресу https://github.com/danya02/rudn-year5-ros-waffleslam**

---

## 1. Цель работы

Познакомиться с основами Robot Operating System (ROS 2), научиться запускать симуляцию мобильного робота TurtleBot 3 в среде Gazebo Harmonic, изучить базовые инструменты взаимодействия с ROS 2 (топики, узлы, сообщения) и получить первичные навыки визуализации данных в RViz 2.

---

## 2. Оборудование и программное обеспечение

- Виртуальная машина Ubuntu 24.04 LTS (Noble) под управлением QEMU/KVM;
- ROS 2 Jazzy Jalisco (LTS-релиз 2024 г.);
- Gazebo Harmonic (gz-sim), интегрированный с ROS 2 через пакет `ros_gz_bridge`;
- Пакеты TurtleBot 3: `turtlebot3`, `turtlebot3_simulations`, `turtlebot3_msgs`;
- Инструменты командной строки ROS 2: `ros2`, `rviz2`, `gz`;
- Язык Rust (rustup stable toolchain), crate `rclrs` 0.7.0;
- `colcon-cargo-ros2` — расширение colcon для сборки Rust-пакетов.

> **Примечание.** Задание предполагает ROS 2 Humble и Gazebo Classic 11, однако работа выполнялась на актуальной версии стека — ROS 2 Jazzy и Gazebo Harmonic. Принципиальных отличий в концепциях нет; команды и пути к пакетам незначительно отличаются.

---

## 3. Теоретическое введение

### 3.1 Что такое ROS 2?

ROS (Robot Operating System) — это гибкая распределённая платформа для разработки программного обеспечения роботов. Она предоставляет аппаратную абстракцию, низкоуровневый контроль устройств, передачу сообщений между процессами и управление пакетами.

**ROS 2** — переработанная версия ROS, использующая DDS (Data Distribution Service) в качестве транспортного уровня вместо централизованного мастер-процесса. Это обеспечивает:
- децентрализованную архитектуру (нет единой точки отказа);
- поддержку систем реального времени;
- улучшенную безопасность и масштабируемость;
- совместимость с распределёнными многомашинными системами.

### 3.2 Ключевые понятия ROS 2

| Понятие | Описание |
|---------|----------|
| **Узел (Node)** | Исполняемый модуль с конкретной задачей. Узлы общаются между собой через топики, сервисы и действия. |
| **Топик (Topic)** | Именованный канал для асинхронного обмена сообщениями по модели «издатель — подписчик». |
| **Сообщение (Message)** | Структура данных, определяющая формат передаваемой информации (например, `geometry_msgs/msg/Twist`). |
| **Сервис (Service)** | Механизм синхронного взаимодействия «запрос — ответ». |
| **Действие (Action)** | Долгосрочная задача с промежуточной обратной связью (надстройка над сервисами). |
| **Пакет (Package)** | Основная единица организации кода: узлы, launch-файлы, конфигурации. |
| **Launch-файл** | Python-скрипт для одновременного запуска нескольких узлов с нужными параметрами. |

### 3.3 TurtleBot 3 и Gazebo

TurtleBot 3 — популярная учебная мобильная робоплатформа. Доступны три модели: Burger, Waffle и Waffle Pi. В данной работе используется модель **Waffle** — дифференциальный привод с лидаром (360°) и стерео-камерой.

Gazebo Harmonic моделирует физику, сенсоры и взаимодействие с окружением. Связь с ROS 2 обеспечивается через `ros_gz_bridge`, который транслирует сообщения между форматами gz и ROS 2.

---

## 4. Ход работы

### 4.1 Настройка окружения

Перед запуском любой команды необходимо инициализировать окружение ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
```

### 4.2 Запуск симуляции TurtleBot 3 в Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

В окне Gazebo появляется окружение `turtlebot3_world` с роботом TurtleBot 3 Waffle.

После запуска в окне Gazebo появляется сцена `turtlebot3_world` с роботом TurtleBot 3 Waffle и характерными стенами-препятствиями.

### 4.3 Изучение топологии ROS 2

**Список активных узлов:**

```bash
ros2 node list
```

Типичный вывод при запущенной симуляции:
```
/gazebo
/robot_state_publisher
/ros_gz_bridge
/transform_listener_impl_...
```

**Список топиков:**

```bash
ros2 topic list
```

Среди топиков можно видеть:
- `/cmd_vel` — команды скорости;
- `/scan` — данные лидара;
- `/odom` — одометрия;
- `/camera/image_raw` — изображение с камеры;
- `/tf`, `/tf_static` — трансформации систем координат.

**Тип топика `/cmd_vel`:**

```bash
ros2 topic info /cmd_vel
```

```
Type: geometry_msgs/msg/TwistStamped
Publisher count: 0
Subscription count: 1
```

> В ROS 2 Jazzy симуляция TurtleBot 3 использует `geometry_msgs/msg/TwistStamped` (с временной меткой) вместо `Twist`, применявшегося в Humble.

**Структура сообщения:**

```bash
ros2 interface show geometry_msgs/msg/TwistStamped
```

```
std_msgs/Header header
    builtin_interfaces/Time stamp
    string frame_id
geometry_msgs/Twist twist
    Vector3 linear
        float64 x   # движение вперёд/назад, м/с
        float64 y   # боковое движение (для omni-drive)
        float64 z
    Vector3 angular
        float64 x
        float64 y
        float64 z   # угловая скорость, рад/с
```


### 4.4 Управление роботом через командную строку

Движение вперёд:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}, frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

Поворот против часовой стрелки:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}, frame_id: ''}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

Остановка (публикация нулевой скорости с частотой 1 Гц):

```bash
ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}, frame_id: ''}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Робот начинает движение вперёд, что подтверждается смещением модели в симуляции.

### 4.5 Визуализация данных в RViz 2

```bash
ros2 run rviz2 rviz2
```

Добавленные отображения:
1. **RobotModel** — топик `/robot_description`, отображает 3D-модель робота;
2. **LaserScan** — топик `/scan`, облако точек лидара;
3. **TF** — дерево трансформаций систем координат.

В RViz видны лучи лидара, обнаруживающие препятствия вокруг робота.

В RViz 2 отображаются: 3D-модель робота, цветное облако точек лидара вокруг него и дерево систем координат.

## 5. Задания для самостоятельной работы

### 5.1 Узел движения по квадрату (`square_movement`)

#### Структура пакета

```
src/square_movement/
├── package.xml      # тип сборки: ament_cargo
├── Cargo.toml       # зависимости: rclrs, geometry_msgs, nav_msgs, std_msgs, builtin_interfaces
└── src/
    └── main.rs      # реализация узла
```

#### Реализация (`src/main.rs`, сокращённо)

```rust
use geometry_msgs::msg::{Twist, TwistStamped, Vector3};
use nav_msgs::msg::Odometry;
use rclrs::{Clock, Context, CreateBasicExecutor, SpinOptions, Time, TimerOptions};
// ... вспомогательные функции (yaw_from_quaternion, angle_turned, elapsed_since) ...

const SPEED: f64 = 0.3;           // м/с
const SIDE_LEN: f64 = 2.0;        // м, длина стороны квадрата
const CORNER_RADIUS: f64 = 0.3;   // м, радиус скругления угла
const TURN_TOLERANCE: f64 = 0.02; // рад, точность остановки поворота (~1°)

#[derive(Debug)]
enum Phase { Straight, Turning, Done }

struct DriveState {
    cmd_vel: rclrs::Publisher<TwistStamped>,
    clock: Clock,
    phase: Phase,
    leg: u32,
    phase_start: Time,
    straight_dur: Duration,
    turn_rate: f64,
    current_heading: Option<f64>, // рад, из /odom
    turn_start_heading: f64,
}

impl DriveState {
    fn tick(&mut self) {
        match self.phase {
            Phase::Done => self.publish(0.0, 0.0),
            Phase::Straight => {
                if elapsed_since(&self.clock, &self.phase_start) < self.straight_dur {
                    self.publish(SPEED, 0.0);
                } else {
                    self.turn_start_heading = self.current_heading.unwrap_or(0.0);
                    self.phase = Phase::Turning;
                    self.phase_start = self.clock.now();
                    self.publish(SPEED, self.turn_rate);
                }
            }
            Phase::Turning => {
                let done = match self.current_heading {
                    Some(h) => angle_turned(h, self.turn_start_heading) >= FRAC_PI_2 - TURN_TOLERANCE,
                    None => elapsed_since(&self.clock, &self.phase_start)
                        >= Duration::from_secs_f64(FRAC_PI_2 / self.turn_rate),
                };
                if done {
                    self.leg += 1;
                    if self.leg >= 4 {
                        self.phase = Phase::Done;
                        self.publish(0.0, 0.0);
                    } else {
                        self.phase = Phase::Straight;
                        self.phase_start = self.clock.now();
                        self.publish(SPEED, 0.0);
                    }
                } else {
                    self.publish(SPEED, self.turn_rate);
                }
            }
        }
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("square_movement_node")?;

    let cmd_vel = node.create_publisher::<TwistStamped>("/cmd_vel")?;
    let worker = node.create_worker::<DriveState>(DriveState::new(cmd_vel, node.get_clock()));

    let _odom_sub = worker.create_subscription(
        "/odom",
        |state: &mut DriveState, msg: Odometry| {
            state.current_heading = Some(yaw_from_quaternion(&msg.pose.pose.orientation));
        },
    )?;

    let _timer = worker.create_timer_repeating(
        TimerOptions::new(Duration::from_millis(50)),
        |state: &mut DriveState| { state.tick(); },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
```

**Принцип работы.** Узел публикует команды в `/cmd_vel` с частотой 20 Гц (таймер 50 мс). Машина состояний переключается между тремя фазами: `Straight` (движение прямо на расчётное время `(SIDE_LEN − 2·CORNER_RADIUS) / SPEED`), `Turning` (поворот со скоростью `SPEED / CORNER_RADIUS` рад/с до достижения угла π/2 по данным одометрии) и `Done` (остановка). Четыре пары «прямо + поворот» образуют замкнутый квадрат со скруглёнными углами. Курс читается из `/odom`; при недоступности одометрии поворот прерывается по расчётному времени.

**Сборка и запуск:**

```bash
cd ~/ros2_waffleslam_ws
source /opt/ros/jazzy/setup.bash
PATH=$PATH:~/.cargo/bin colcon build --packages-select square_movement
source install/setup.bash
ros2 run square_movement square_movement
```

Робот последовательно проходит четыре стороны квадрата со скруглёнными углами (радиус 0,3 м) и останавливается, вернувшись в исходную точку.

### 5.2 Узел объезда препятствий (`basic_movement`)

#### Структура пакета

```
src/basic_movement/
├── package.xml      # тип сборки: ament_cargo
├── Cargo.toml       # зависимости: rclrs, geometry_msgs, sensor_msgs, builtin_interfaces
└── src/
    └── main.rs      # реализация узла
```

#### Реализация (`src/main.rs`, сокращённо)

```rust
use rclrs::{Context, CreateBasicExecutor, PublisherOptions, SpinOptions, SubscriptionOptions};
use std::sync::Arc;

const FRONT_ANGLE: f32 = 30.0;   // градусов в каждую сторону от курса
const STOP_DISTANCE: f32 = 0.5;  // м — расстояние начала поворота
const LINEAR_SPEED: f64 = 0.7;   // м/с
const TURN_SPEED: f64 = 0.8;     // рад/с

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("obstacle_avoider")?;

    let publisher = node.create_publisher(PublisherOptions::new("cmd_vel"))?;
    let clock = node.get_clock();

    let _subscription = {
        let publisher = Arc::clone(&publisher);
        node.create_subscription(
            SubscriptionOptions::new("scan"),
            move |msg: sensor_msgs::msg::LaserScan| {
                let front_rad = FRONT_ANGLE.to_radians();
                let mut min_dist = f32::INFINITY;
                for (i, &r) in msg.ranges.iter().enumerate() {
                    let angle = msg.angle_min + i as f32 * msg.angle_increment;
                    if angle.abs() <= front_rad && msg.range_min < r && r < msg.range_max {
                        if r < min_dist { min_dist = r; }
                    }
                }
                let mut cmd = geometry_msgs::msg::TwistStamped::default();
                if min_dist < STOP_DISTANCE {
                    cmd.twist.angular.z = TURN_SPEED;
                } else {
                    cmd.twist.linear.x = LINEAR_SPEED;
                }
                let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap();
                cmd.header.stamp = builtin_interfaces::msg::Time { sec, nanosec };
                let _ = publisher.publish(cmd);
            },
        )?
    };

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
```

**Принцип работы.** При каждом поступлении сообщения от лидара колбэк находит минимальное расстояние в секторе ±30° от прямого курса. Если препятствие ближе 0,5 м — публикуется `TwistStamped` с угловой скоростью 0,8 рад/с; иначе — команда движения вперёд 0,7 м/с. Worker не используется: узел реализован через замыкание подписки, захватывающее `Arc` на publisher и `clock`.

**Сборка и запуск:**

```bash
cd ~/ros2_waffleslam_ws
source /opt/ros/jazzy/setup.bash
PATH=$PATH:~/.cargo/bin colcon build --packages-select basic_movement
source install/setup.bash
ros2 run basic_movement basic_movement
```

Узел обнаруживает стену ближе 0,5 м и начинает поворот; после освобождения сектора снова переходит в движение вперёд.

## 6. Ответы на контрольные вопросы

**1. Что такое ROS 2 и в чём его отличие от ROS 1?**

ROS 2 — обновлённая распределённая платформа для разработки роботизированных систем. Ключевые отличия от ROS 1:
- Отсутствие центрального процесса `rosmaster` — узлы общаются напрямую через DDS;
- Поддержка систем жёсткого реального времени;
- Нативная поддержка Windows и macOS;
- Улучшенная безопасность (шифрование на уровне транспорта через DDS-Security);
- Стабильный API и политика обратной совместимости (LTS-релизы).

**2. Какие основные компоненты ROS 2 вы использовали в работе?**

- Узлы: `gazebo`, `robot_state_publisher`, `ros_gz_bridge`, `square_movement_node`, `obstacle_avoider`;
- Топики: `/cmd_vel`, `/scan`, `/odom`, `/tf`;
- Сообщения: `TwistStamped`, `LaserScan`;
- Launch-файлы: `turtlebot3_world.launch.py`;
- Инструменты: `ros2 topic pub/list/info/echo`, `rviz2`;
- Библиотека: `rclrs` (Rust-биндинг ROS 2 Client Library).

**3. Что такое топик в ROS 2? Приведите примеры топиков из работы.**

Топик — именованный канал для асинхронного обмена сообщениями по модели «издатель — подписчик». Несколько узлов могут одновременно публиковать и читать из одного топика.

Примеры из работы:
- `/cmd_vel` (TwistStamped) — команды скорости для робота;
- `/scan` (LaserScan) — 360°-скан лидара с расстояниями до препятствий;
- `/odom` (Odometry) — оценка положения и скорости по одометрии;
- `/tf` — трансформации между системами координат.

**4. Какой тип сообщения используется для управления движением TurtleBot 3? Опишите его структуру.**

В ROS 2 Jazzy используется `geometry_msgs/msg/TwistStamped`. Он содержит:
- `header` (временная метка и имя системы координат);
- `twist.linear` (вектор линейной скорости в м/с: `x` — вперёд/назад, `y` — вбок);
- `twist.angular` (вектор угловой скорости в рад/с: `z` — рысканье).

**5. Для чего нужен RViz 2? Какие данные вы визуализировали?**

RViz 2 — инструмент 3D-визуализации для ROS 2. Позволяет отображать модель робота, сенсорные данные, карты и планы движения в единой сцене. В работе визуализировались: 3D-модель робота (`RobotModel`), данные лидара (`LaserScan`) и дерево трансформаций (`TF`).

**6. Как запустить симуляцию TurtleBot 3 с другой моделью робота?**

```bash
export TURTLEBOT3_MODEL=burger   # или waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Модель `burger` — двухколёсная платформа только с лидаром (нет камеры). `waffle_pi` аналогична `waffle`, но с камерой Raspberry Pi.

**7. Как остановить движение робота программно?**

Опубликовать команду с нулевыми скоростями:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}, frame_id: ''}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Или из Rust-узла вызвать `publisher.publish(TwistStamped::default())` при завершении работы.

---

## 7. Выводы

В ходе лабораторной работы было изучено:

1. **Архитектура ROS 2**: узлы, топики и сообщения — три ключевых примитива для создания распределённых роботизированных систем. DDS-транспорт обеспечивает надёжную доставку данных без единой точки отказа.

2. **Запуск симуляции**: launch-файлы позволяют одной командой запустить всю сложную систему (Gazebo, robot_state_publisher, ros_gz_bridge) с нужными параметрами.

3. **Практическое программирование**: реализованы два узла на Rust с использованием `rclrs` — движение по квадрату (конечный автомат с обратной связью по одометрии, скруглёнными углами и Worker-паттерном) и объезд препятствий (реактивное управление через замыкание подписки).

4. **Особенность ROS 2 Jazzy**: в отличие от Humble, симуляционный бридж TurtleBot 3 использует `TwistStamped` вместо `Twist`, что потребовало адаптации кода управления.

Основная трудность — согласование формата команд скорости (`TwistStamped` вместо ожидаемого `Twist`). Эта несовместимость была обнаружена анализом конфигурации бриджа и устранена соответствующей настройкой Nav2.
