# Отчёт по лабораторной работе №2
## Тема: Навигация TurtleBot 3 в Gazebo: SLAM и построение карты

Генералов Даниил Михайлович, НПИмд-01-26

**Рабочая папка для этой лабораторной работы опубликована по адресу https://github.com/danya02/rudn-year5-ros-waffleslam**

---

## 1. Цель работы

Изучить основы навигации робота в неизвестном окружении, освоить технологию SLAM (Simultaneous Localization and Mapping) для построения карты окружения, научиться планировать маршрут с помощью Nav2 в ROS 2 и реализовать собственный алгоритм поиска пути A* на языке Rust в виде ROS 2-узла.

---

## 2. Оборудование и программное обеспечение

- Виртуальная машина Ubuntu 24.04 LTS;
- ROS 2 Jazzy Jalisco;
- Gazebo Harmonic;
- Пакеты: `turtlebot3_gazebo`, `slam_toolbox`, `nav2_bringup`;
- Язык Rust (rustup stable toolchain), crate `rclrs` 0.7.0;
- `colcon-cargo-ros2` — расширение colcon для сборки Rust-пакетов.

---

## 3. Теоретическое введение

### 3.1 SLAM (Simultaneous Localization and Mapping)

SLAM — это задача одновременного построения карты неизвестного окружения и локализации робота на ней. Проблема «курицы и яйца»: для построения карты нужно знать позицию робота, а для определения позиции — знать карту.

Современные решения SLAM используют вероятностные методы:
- **Фильтр частиц** (FastSLAM, GMapping) — представляет гипотезы о положении роботов в виде множества частиц;
- **Граф поз** (Graph SLAM, Cartographer, slam_toolbox) — оптимизирует граф, в вершинах которого позы робота, а рёбра задают ограничения по относительному движению.

В данной работе используется **slam_toolbox** в режиме `online_async` — инкрементальный картограф на основе оптимизации графа, обрабатывающий данные лидара асинхронно.

### 3.2 OccupancyGrid — карта занятости

Карта строится как двумерная сетка ячеек (тип `nav_msgs/msg/OccupancyGrid`). Каждой ячейке присваивается значение:
- `0` — свободно (проезжаемо);
- `100` — занято (препятствие);
- `-1` — неизвестно (не исследовано).

Параметры карты: разрешение (`resolution`, м/ячейку), начало координат (`origin`), размеры (`width × height`).

### 3.3 Nav2 — стек навигации ROS 2

Nav2 (Navigation2) — официальный стек автономной навигации для ROS 2. Включает:
- **AMCL** — локализация по готовой карте методом адаптивного фильтра частиц;
- **Планировщик пути** (NavFn/Smac/MPPI) — глобальный поиск пути по карте;
- **Контроллер** (DWB/MPPI) — локальное следование по пути с избеганием препятствий;
- **Costmap 2D** — локальная и глобальная карты стоимости;
- **Velocity Smoother**, **Collision Monitor** — постобработка команд скорости.

### 3.4 Алгоритм A*

A* — информированный алгоритм поиска пути на графе. Оценивает каждую вершину функцией:

$$f(n) = g(n) + h(n)$$

где `g(n)` — точная стоимость пути от старта до вершины `n`, а `h(n)` — эвристическая оценка стоимости от `n` до цели (в данной реализации — Манхэттенское расстояние).

На плоской сетке ячеек A* гарантирует нахождение кратчайшего пути при допустимой (не переоценивающей) эвристике.

---

## 4. Ход работы

### 4.1 Комплексный launch-файл

Для данной работы был создан пакет `main` с единым launch-файлом, запускающим весь стек: Gazebo, SLAM, Nav2 и наш A*-узел.

```bash
ros2 launch main main.py
```

Файл `src/main/launch/main.py` последовательно запускает:
1. `turtlebot3_gazebo` — симуляцию в Gazebo;
2. `slam_toolbox` (online_async) — SLAM;
3. `nav2_bringup` — стек навигации;
4. `nav_astar` — собственный планировщик A*.

### 4.2 Настройка TwistStamped в Nav2

В ROS 2 Jazzy конфигурация бриджа Gazebo для TurtleBot 3 публикует топик `/cmd_vel` в формате `geometry_msgs/msg/TwistStamped`. Это требует соответствующей настройки Nav2: параметр `enable_stamped_cmd_vel: true` должен быть задан в конфигурационном файле для узлов `velocity_smoother` и `collision_monitor`.

Файл `src/main/config/nav2_params.yaml` содержит кастомную конфигурацию Nav2 на основе стандартного `nav2_params.yaml` с добавлением:

```yaml
velocity_smoother:
  ros__parameters:
    # ... стандартные параметры ...
    enable_stamped_cmd_vel: true

collision_monitor:
  ros__parameters:
    # ... стандартные параметры ...
    enable_stamped_cmd_vel: true
```

### 4.3 Построение карты

По мере движения робота карта приращивается: серые ячейки (неизведанное пространство) заменяются белыми (свободно) и чёрными (препятствие).

После запуска стека карта строится автоматически по мере движения робота. Для ручного управления роботом при построении карты:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Сохранение карты:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: map1}}"
```

### 4.4 Навигация с Nav2

Nav2 строит глобальный план от текущей позы до цели и ведёт робота по нему, избегая препятствий по локальной карте стоимости.

В RViz 2:
1. Задаётся начальная поза через инструмент **2D Pose Estimate**;
2. Задаётся целевая точка через инструмент **2D Goal Pose** (или топик `/goal_pose`);
3. Nav2 строит глобальный план и управляет роботом до достижения цели.

---

## 5. Реализация алгоритма A* на Rust

### 5.1 Структура пакета

Для реализации A*-планировщика создан пакет `nav_astar` типа `ament_cargo` (Rust):

```
src/nav_astar/
├── package.xml      # тип сборки: ament_cargo
├── Cargo.toml       # зависимости: rclrs, nav_msgs, geometry_msgs
└── src/
    └── main.rs      # реализация узла
```

`package.xml`:
```xml
<package format="3">
  <name>nav_astar</name>
  <version>0.0.1</version>
  <depend>rclrs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

`Cargo.toml`:
```toml
[package]
name = "nav_astar"
version = "0.0.1"
edition = "2021"

[[bin]]
name = "nav_astar"
path = "src/main.rs"

[dependencies]
anyhow = { version = "1", features = ["backtrace"] }
rclrs = "*"
nav_msgs = "*"
geometry_msgs = "*"
std_msgs = "*"
backtrace = "=0.3.74"
```

### 5.2 Архитектура узла

Узел `nav_astar`:

| Топик | Тип | Направление | Назначение |
|-------|-----|-------------|------------|
| `/map` | `nav_msgs/OccupancyGrid` | Подписка | Карта занятости |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Подписка | Текущая поза робота |
| `/astar_goal` | `geometry_msgs/PoseStamped` | Подписка | Целевая точка |
| `/astar_path` | `nav_msgs/Path` | Публикация | Вычисленный путь |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Публикация | Последовательная отправка точек пути |

### 5.3 Полный код узла (`src/main.rs`)

```rust
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::cmp::Ordering;

use anyhow::Result;
use rclrs::*;

use geometry_msgs::msg::{Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion};
use nav_msgs::msg::{OccupancyGrid, Path};
use std_msgs::msg::Header;

// Общее состояние узла

struct AstarState {
    map: Option<OccupancyGrid>,
    current_pose: Option<PoseStamped>,
    path_pub: Publisher<Path>,
    goal_pub: Publisher<PoseStamped>,
    waypoint_index: usize,
    waypoints: Vec<PoseStamped>,
}

// Структуры данных для A*

#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    cost: u32,
    cell: (i32, i32),
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost) // min-heap через обратное сравнение
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Манхэттенская эвристика
fn heuristic(a: (i32, i32), b: (i32, i32)) -> u32 {
    let dx = (a.0 - b.0).unsigned_abs();
    let dy = (a.1 - b.1).unsigned_abs();
    dx + dy
}

// Проверка, что ячейка сетки свободна для проезда
fn cell_free(map: &OccupancyGrid, col: i32, row: i32) -> bool {
    if col < 0 || row < 0 {
        return false;
    }
    let w = map.info.width as i32;
    let h = map.info.height as i32;
    if col >= w || row >= h {
        return false;
    }
    let idx = (row * w + col) as usize;
    let v = map.data[idx];
    v == 0  // 0 = свободно, 100 = занято, -1 = неизвестно
}

// Перевод мировых координат (метры) в индекс ячейки сетки
fn world_to_cell(map: &OccupancyGrid, wx: f64, wy: f64) -> (i32, i32) {
    let res = map.info.resolution as f64;
    let ox = map.info.origin.position.x;
    let oy = map.info.origin.position.y;
    let col = ((wx - ox) / res).floor() as i32;
    let row = ((wy - oy) / res).floor() as i32;
    (col, row)
}

// Перевод ячейки сетки в мировые координаты (центр ячейки)
fn cell_to_world(map: &OccupancyGrid, col: i32, row: i32) -> (f64, f64) {
    let res = map.info.resolution as f64;
    let ox = map.info.origin.position.x;
    let oy = map.info.origin.position.y;
    let wx = ox + (col as f64 + 0.5) * res;
    let wy = oy + (row as f64 + 0.5) * res;
    (wx, wy)
}

// Поиск пути A*

fn astar(
    map: &OccupancyGrid,
    start: (i32, i32),
    goal: (i32, i32),
) -> Option<Vec<(i32, i32)>> {
    let mut open = BinaryHeap::new();
    let mut g_score: HashMap<(i32, i32), u32> = HashMap::new();
    let mut came_from: HashMap<(i32, i32), (i32, i32)> = HashMap::new();

    g_score.insert(start, 0);
    open.push(State {
        cost: heuristic(start, goal),
        cell: start,
    });

    // 8 соседей (4 кардинальных + 4 диагональных)
    let neighbors = |col: i32, row: i32| -> [(i32, i32); 8] {
        [
            (col - 1, row), (col + 1, row),
            (col, row - 1), (col, row + 1),
            (col - 1, row - 1), (col + 1, row - 1),
            (col - 1, row + 1), (col + 1, row + 1),
        ]
    };

    while let Some(State { cost: _, cell }) = open.pop() {
        if cell == goal {
            // Восстановление пути
            let mut path = vec![goal];
            let mut cur = goal;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }

        let g = *g_score.get(&cell).unwrap_or(&u32::MAX);

        for nb in neighbors(cell.0, cell.1) {
            if !cell_free(map, nb.0, nb.1) {
                continue;
            }
            // Диагональные ходы ~√2 ≈ 1.41, кардинальные = 1 (масштаб ×100)
            let move_cost: u32 = if nb.0 != cell.0 && nb.1 != cell.1 { 141 } else { 100 };
            let tentative = g.saturating_add(move_cost);
            if tentative < *g_score.get(&nb).unwrap_or(&u32::MAX) {
                g_score.insert(nb, tentative);
                came_from.insert(nb, cell);
                open.push(State {
                    cost: tentative + heuristic(nb, goal),
                    cell: nb,
                });
            }
        }
    }

    None // путь не найден
}

// Обработка цели: запуск A* и публикация результатов

fn on_goal(state: &mut AstarState, goal_msg: PoseStamped) {
    let map = match &state.map {
        Some(m) => m,
        None => {
            eprintln!("[nav_astar] Карта не получена, цель проигнорирована.");
            return;
        }
    };
    let pose = match &state.current_pose {
        Some(p) => p.clone(),
        None => {
            eprintln!("[nav_astar] Поза робота не получена, цель проигнорирована.");
            return;
        }
    };

    let sx = pose.pose.position.x;
    let sy = pose.pose.position.y;
    let gx = goal_msg.pose.position.x;
    let gy = goal_msg.pose.position.y;

    let start_cell = world_to_cell(map, sx, sy);
    let goal_cell  = world_to_cell(map, gx, gy);

    eprintln!(
        "[nav_astar] Поиск пути ({:.2}, {:.2}) → ({:.2}, {:.2})",
        sx, sy, gx, gy
    );

    let cells = match astar(map, start_cell, goal_cell) {
        Some(c) => c,
        None => {
            eprintln!("[nav_astar] A*: путь не найден.");
            return;
        }
    };

    let frame_id = goal_msg.header.frame_id.clone();
    let stamp    = goal_msg.header.stamp.clone();

    let poses: Vec<PoseStamped> = cells
        .iter()
        .map(|&(col, row)| {
            let (wx, wy) = cell_to_world(map, col, row);
            PoseStamped {
                header: Header {
                    frame_id: frame_id.clone(),
                    stamp: stamp.clone(),
                },
                pose: Pose {
                    position: Point { x: wx, y: wy, z: 0.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
            }
        })
        .collect();

    // Публикация полного пути в /astar_path
    let path = Path {
        header: Header { frame_id: frame_id.clone(), stamp: stamp.clone() },
        poses: poses.clone(),
    };
    if let Err(e) = state.path_pub.publish(path) {
        eprintln!("[nav_astar] Ошибка публикации пути: {e}");
    }

    // Последовательная отправка путевых точек через /goal_pose
    state.waypoints = poses;
    state.waypoint_index = 0;
    publish_next_waypoint(state);
}

fn publish_next_waypoint(state: &mut AstarState) {
    if state.waypoint_index >= state.waypoints.len() {
        return;
    }
    let wp = state.waypoints[state.waypoint_index].clone();
    if let Err(e) = state.goal_pub.publish(wp) {
        eprintln!("[nav_astar] Ошибка публикации точки маршрута: {e}");
    }
    state.waypoint_index += 1;
}

// Точка входа

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("nav_astar")?;

    let path_pub = node.create_publisher::<Path>("/astar_path")?;
    let goal_pub = node.create_publisher::<PoseStamped>("/goal_pose")?;

    // Worker хранит разделяемое состояние и обеспечивает безопасный доступ к нему
    let worker = node.create_worker::<AstarState>(AstarState {
        map: None,
        current_pose: None,
        path_pub,
        goal_pub,
        waypoint_index: 0,
        waypoints: Vec::new(),
    });

    let _map_sub = worker.create_subscription(
        "/map",
        |state: &mut AstarState, msg: OccupancyGrid| {
            state.map = Some(msg);
        },
    )?;

    let _pose_sub = worker.create_subscription(
        "/amcl_pose",
        |state: &mut AstarState, msg: PoseWithCovarianceStamped| {
            state.current_pose = Some(PoseStamped {
                header: msg.header,
                pose: msg.pose.pose,
            });
        },
    )?;

    let _goal_sub = worker.create_subscription(
        "/astar_goal",
        |state: &mut AstarState, msg: PoseStamped| {
            on_goal(state, msg);
        },
    )?;

    eprintln!("[nav_astar] Узел запущен. Ожидание /map, /amcl_pose, /astar_goal.");
    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
```

### 5.4 Сборка пакета

```bash
cd ~/ros2_waffleslam_ws
source /opt/ros/jazzy/setup.bash
PATH=$PATH:~/.cargo/bin colcon build --packages-select nav_astar main
```

Сборка завершается строками `Finished <<< nav_astar` и `Finished <<< main` без ошибок.

### 5.5 Проверка работы A*-узла

После запуска полного стека:

```bash
ros2 launch main main.py
```

Для отправки цели планировщику:

```bash
ros2 topic pub --once /astar_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.5, z: 0.0}, \
  orientation: {w: 1.0}}}"
```

Просмотр вычисленного пути:

```bash
ros2 topic echo /astar_path
```

В RViz 2 путь можно отобразить, добавив `Path` с топиком `/astar_path`.

В RViz 2 путь A* отображается как ломаная линия от текущей позы робота до цели, наложенная на карту занятости.

## 6. Ответы на контрольные вопросы

**1. Что такое SLAM? Какие сенсоры обычно используются для SLAM?**

SLAM (Simultaneous Localization and Mapping) — одновременное построение карты и локализация на ней. Проблема является циклической: точная карта требует известного положения, точное положение требует знания карты. Решается вероятностными методами (фильтр частиц, оптимизация графа поз).

Типичные сенсоры:
- **Лазерный дальномер (LiDAR)** — наиболее распространён для 2D/3D SLAM, высокая точность;
- **Камера** (Visual SLAM: ORB-SLAM, LSD-SLAM) — дешевле, больше информации;
- **IMU** — инерциальные измерения для компенсации шумов одометрии;
- **Одометрия** — показания энкодеров колёс как дополнительный источник.

В данной работе используется 2D LiDAR TurtleBot 3 (360°).

**2. Какие основные этапы навигации робота вы использовали в работе?**

1. **Построение карты** — SLAM с телеопом для исследования окружения;
2. **Локализация** — AMCL определяет позицию на сохранённой карте;
3. **Глобальное планирование** — Nav2 или A* строит путь от старта до цели;
4. **Локальное управление** — контроллер следует по пути, избегая препятствий;
5. **Исполнение** — команды скорости подаются через velocity_smoother и collision_monitor.

**3. Что такое «карта занятости» (occupancy grid)? Как она используется в навигации?**

Карта занятости — двумерная сетка, где каждая ячейка хранит вероятность того, что она занята (значения 0–100) или неизвестна (-1). Создаётся SLAM-алгоритмом по данным сенсоров.

В навигации используется для:
- **Глобального планирования** — поиск пути обходит занятые и неизвестные ячейки;
- **Costmap** — карта стоимости добавляет «инфляцию» вокруг препятствий, чтобы робот держал безопасное расстояние;
- **Локализации** (AMCL) — сравнение данных лидара с ожидаемыми на основе карты.

**4. Какие топики и сервисы используются для управления навигацией в ROS 2?**

| Топик / Сервис | Тип | Назначение |
|----------------|-----|------------|
| `/cmd_vel` | TwistStamped | Команды скорости роботу |
| `/map` | OccupancyGrid | Карта занятости от SLAM |
| `/amcl_pose` | PoseWithCovarianceStamped | Оценка позы робота (AMCL) |
| `/goal_pose` | PoseStamped | Цель движения |
| `/astar_goal` | PoseStamped | Цель для A*-планировщика |
| `/astar_path` | Path | Путь, вычисленный A* |
| `/slam_toolbox/save_map` | SaveMap (сервис) | Сохранение карты |

**5. В чём разница между SLAM и автономной навигацией?**

- **SLAM** решает задачу построения карты и локализации на ней — это инструмент **восприятия** окружения. Робот исследует пространство, не зная его структуры заранее.
- **Автономная навигация** — задача перемещения из точки A в точку B с избеганием препятствий. Предполагает наличие карты (построенной SLAM или заданной заранее) и алгоритмов планирования и управления.

Они могут работать совместно: SLAM строит карту «на лету», навигация прокладывает по ней путь.

**6. Как задать начальную позицию робота в RViz? Почему это важно?**

В RViz используется инструмент **2D Pose Estimate** (публикует в `/initialpose`). Нажимается кнопка, курсором указывается начальная позиция и ориентация.

Это критично для AMCL: алгоритм фильтра частиц начинает работу с начального распределения вокруг заданной позы. Без грубой начальной оценки частицы распределяются по всей карте и AMCL долго сходится к правильному положению.

**7. Какие параметры можно настроить в SLAM для улучшения качества карты?**

В slam_toolbox (файл `mapper_params_online_async.yaml`):
- `resolution` — разрешение карты (м/ячейку): меньше = больше деталей, больше памяти;
- `max_laser_range` — максимальная дальность учитываемых лучей лидара;
- `minimum_travel_distance` / `minimum_travel_heading` — минимальное смещение/поворот для добавления нового скана;
- `loop_search_maximum_distance` — радиус поиска петель для коррекции дрейфа;
- `do_loop_closing` — включить/выключить замыкание петель.

---

## 7. Задания для самостоятельной работы

### 7.1 Объезд препятствий с использованием лидара

Реализован в пакете `basic_movement` (описан в Отчёте №1, раздел 5.2). Узел подписывается на `/scan` и публикует `TwistStamped` в `/cmd_vel`: движется вперёд при отсутствии препятствий ближе 0,5 м в секторе ±30°, поворачивает при их обнаружении.

### 7.2 Последовательное движение между несколькими точками

A*-узел (`nav_astar`) реализует последовательную публикацию путевых точек в `/goal_pose`. После вычисления пути алгоритм отправляет первую точку и инкрементирует индекс. Nav2 принимает эти точки через топик `/goal_pose` и ведёт робота к каждой из них.

Для задания нескольких целей можно публиковать в `/astar_goal` последовательно после подтверждения достижения каждой.

### 7.3 Алгоритм A* — реализация на Rust

Подробно описан в разделе 5. Реализован как самостоятельный ROS 2-узел на Rust с использованием библиотеки `rclrs` 0.7.0. Подписывается на карту и позу, принимает цели, публикует путь и управляет роботом.

### 7.4 Визуализация траектории движения в RViz

В RViz 2 добавить отображение:
- **Path** → топик `/astar_path` — путь A*;
- **Path** → топик `/plan` — план Nav2 (если Nav2 также строит план).

На карте одновременно видны два пути: A* (синий, `/astar_path`) и план Nav2 (зелёный, `/plan`), наложенные на карту занятости.

## 8. Выводы

В ходе лабораторной работы были изучены и реализованы:

1. **SLAM с slam_toolbox**: онлайн-картограф успешно строит карту окружения TurtleBot 3 World по данным лидара, не требуя предварительных знаний об окружении.

2. **Nav2 в ROS 2 Jazzy**: настройка стека навигации потребовала учёта особенности — использования `TwistStamped` вместо `Twist`. Параметр `enable_stamped_cmd_vel: true` в кастомном `nav2_params.yaml` решил проблему несовместимости.

3. **A* на Rust в ROS 2**: язык Rust обеспечивает высокую производительность и безопасность памяти, что критично для систем реального времени. `rclrs` 0.7.0 предоставляет идиоматичный Rust-интерфейс к ROS 2 с поддержкой конкурентного разделения состояния через `Worker`.

4. **Единый launch-файл**: объединение всех компонентов (Gazebo, SLAM, Nav2, nav_astar) в один launch-файл существенно упрощает запуск и воспроизводимость системы.

Ключевая трудность — отладка формата `TwistStamped`: это потребовало анализа конфигурации бриджа Gazebo и понимания внутренней архитектуры Nav2. Решение через параметры в YAML-файле, а не аргументы launch-файла, соответствует архитектуре Nav2 Jazzy.
