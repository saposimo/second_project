# Stato lavoro - second_project

Questo file riassume lo stato del progetto ROS2 `second_project` dopo il lavoro fatto su mapping e navigation. Serve come memoria operativa per riprendere il lavoro o per altri agenti AI.

## Contesto ambiente

- Codice locale Mac: `/Users/simone/ROS2/ros2_projects/second_project_ws/src/second_project`
- VM host: `/home/simone/robotics-ros2`
- Container robotics:
  - workspace: `/home/robotics/colcon_ws`
  - package: `/home/robotics/colcon_ws/src/second_project`
  - bag: `/home/robotics/bags/rosbag2_2026_05_20-11_03_32/`
- Il container monta:
  - `/home/simone/robotics-ros2/colcon_ws -> /home/robotics/colcon_ws`
  - `/home/simone/robotics-ros2/bags -> /home/robotics/bags`

## Consegna

La consegna richiede:

- Task 1: mapping usando il bag, conversione PointCloud2 -> LaserScan, SLAM, RViz configurato con mappa, lidar e TF.
- Task 2: navigazione usando Stage, robot AgileX Scout Mini Omnidirectional, Nav2, goal publisher C++ che legge `second_project/csv/goals.csv`.
- Cartella `map/` con PNG della mappa ricostruita e serialized map se si usa `slam_toolbox`.
- Package chiamato `second_project`.
- Niente path assoluti.
- Progetto in C/C++ per i nodi.
- Non consegnare `build/`, `install/`, `log/`, bag, cartelle di appunti o zip.

Trascrizione consegna creata in:

- `CONSEGNA_SECOND_PROJECT_2026.md`

## Stato Task 1 - Mapping

Architettura attuale:

```text
/ugv/rslidar_points
  -> pointcloud_to_laserscan_node
  -> /scan_raw
  -> scan_qos_relay
  -> /scan
  -> slam_toolbox
  -> /map
```

Nodi nel launch:

- `pointcloud_to_laserscan_node`: converte `PointCloud2` in `LaserScan`.
- `scan_qos_relay`: relay QoS da `/scan_raw` best-effort a `/scan` reliable.
- `async_slam_toolbox_node`: SLAM in mapping mode.
- `rviz2`: config mapping.

File principali:

- `launch/mapping.launch.py`
- `config/slam_toolbox_params.yaml`
- `config/mapping_rviz.rviz`
- `src/scan_qos_relay.cpp`

Motivo del relay QoS:

- `pointcloud_to_laserscan` pubblica dati sensore con QoS best-effort.
- `slam_toolbox` si connette meglio con `/scan` reliable in questa configurazione.
- Quindi il nodo di conversione vero è 1, ma la pipeline dati prima di SLAM usa 2 nodi.

Comandi mapping in VM/container:

```bash
cd ~/colcon_ws
colcon build --packages-select second_project
source install/setup.bash
QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 launch second_project mapping.launch.py
```

In un secondo terminale/container:

```bash
cd ~/colcon_ws
source install/setup.bash
ros2 bag play ~/bags/rosbag2_2026_05_20-11_03_32/ --clock --rate 0.5
```

Salvataggio mappa:

```bash
cd ~/colcon_ws
source install/setup.bash
mkdir -p ~/colcon_ws/src/second_project/map
ros2 run nav2_map_server map_saver_cli -f ~/colcon_ws/src/second_project/map/map
```

Se viene salvata come `.pgm` ma si vuole usare `map.png`:

```bash
cd ~/colcon_ws/src/second_project/map
mv map.pgm map.png
sed -i 's/^image:.*/image: map.png/' map.yaml
```

Salvataggio posegraph SLAM, con mapping ancora attivo:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/robotics/colcon_ws/src/second_project/map/map'}"
```

## Stato mappa

La mappa salvata in precedenza era:

- `map/map.png`
- `map/map.yaml`

Contenuto atteso `map.yaml`:

```yaml
image: map.png
mode: trinary
resolution: 0.05
origin: [-6.78, -12.7, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

Dimensione mappa vista nei log: circa `612 x 350`, quindi dimensione Stage:

```text
W = 612 * 0.05 = 30.6 m
H = 350 * 0.05 = 17.5 m
```

Nel world file il floorplan e centrato rispetto all'origine mappa:

```text
pose [ 8.52 -3.95 0.0 0.0 ]
size [ 30.6 17.5 0.5 ]
```

## Stato Task 2 - Navigation

La navigation e stata riallineata ai codici di Claudio:

- `Code/stage_ros2_stageros`
- `Code/stage_nav2_showcase`
- `Code/stage_slam_toolbox_showcase`

Architettura attuale:

```text
Stage / stage_ros2_stageros
  -> /clock
  -> /base_scan
  -> /odom
  -> /tf

Nav2 map_server
  -> /map

AMCL
  -> map -> odom

Nav2 navigation_launch.py
  -> planner/controller/bt_navigator/costmaps

goal_publisher
  -> /navigate_to_pose action
```

File principali:

- `launch/navigation.launch.py`
- `config/nav2_params.yaml`
- `config/navigation_rviz.rviz`
- `worlds/second_project.world`
- `src/goal_publisher.cpp`

Topic/frame usati in navigation:

- LaserScan Stage: `/base_scan`
- Odometry: `/odom`
- Velocity command: `/cmd_vel`
- Clock: `/clock`
- Frame global: `map`
- Odom frame: `odom`
- Robot base: `base_footprint`
- TF atteso: `map -> odom -> base_footprint -> base_link -> base_laser_link`

`goal_publisher.cpp`:

- Legge `csv/goals.csv`.
- Invia goal all'action `navigate_to_pose`.
- Manda il prossimo goal quando il precedente termina, viene abortito o cancellato.
- Usa parametri:
  - `action_name`
  - `frame_id`
  - `csv_path`

## Stato Stage

Problema corrente importante:

- Nella VM c'e una cartella `stage_ros2` clonata da TUW:

```bash
git clone -b humble https://github.com/tuw-robotics/stage_ros2.git
```

- Quel package ha nome:

```xml
<name>stage_ros2</name>
```

- Non e quello usato dai codici di Claudio.
- Il progetto ora richiede:

```xml
<name>stage_ros2_stageros</name>
```

Package richiesto:

```text
stage_ros2_stageros
```

Deve stare in:

```text
~/colcon_ws/src/stage_ros2_stageros
```

Il codice di Claudio originale era nello zip:

```text
stage_ros2_stageros.zip
```

Da fare:

1. Trovare/copiare `stage_ros2_stageros.zip` nella VM.
2. Estrarlo in `~/colcon_ws/src`.
3. Disattivare il vecchio TUW `stage_ros2`.

Comandi previsti:

```bash
cd ~/colcon_ws/src
mv stage_ros2 ~/stage_ros2_tuw_disabled 2>/dev/null
unzip stage_ros2_stageros.zip
cd ~/colcon_ws/src/stage_ros2_stageros
grep -n "<name>" package.xml
```

Deve stampare:

```text
<name>stage_ros2_stageros</name>
```

Build:

```bash
cd ~/colcon_ws
colcon build --packages-select stage_ros2_stageros second_project
source install/setup.bash
ros2 pkg list | grep stage
```

## Comandi per avviare navigation

Dopo aver installato `stage_ros2_stageros`:

```bash
cd ~/colcon_ws
source install/setup.bash
QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 launch second_project navigation.launch.py
```

Debug senza RViz:

```bash
ros2 launch second_project navigation.launch.py rviz:=false
```

Debug senza goal publisher:

```bash
ros2 launch second_project navigation.launch.py use_goal_publisher:=false
```

Controlli runtime utili:

```bash
ros2 topic list | grep -E "base_scan|odom|cmd_vel|map|clock"
ros2 topic echo /base_scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
```

## File modificati

Modifiche principali fatte:

- `.gitignore`
  - Esclude `Note by Claudio/`
  - Esclude `Code by Claudio/`
- `launch/mapping.launch.py`
  - Refactor stile Claudio con launch arguments e `OpaqueFunction`.
  - Mantiene conversione PointCloud2 -> LaserScan per il bag.
- `launch/navigation.launch.py`
  - Usa `stage_ros2_stageros/stage.launch.py`.
  - Avvia `map_server`, `amcl`, `lifecycle_manager_localization`.
  - Include `nav2_bringup/launch/navigation_launch.py`.
  - Avvia `goal_publisher` e RViz.
- `config/nav2_params.yaml`
  - Rifatto sul modello `stage_nav2_showcase`.
  - Usa `/base_scan` e `base_footprint`.
  - Mantiene configurazione omnidirezionale.
- `config/slam_toolbox_params.yaml`
  - Reso piu simile allo showcase SLAM.
  - Frame del bag mantenuti: `UGV_odom`, `UGV_base_link`, `/scan`.
- `config/navigation_rviz.rviz`
  - LaserScan su `/base_scan`.
  - Rimosso RobotModel perché Stage non pubblica `/robot_description`.
- `config/mapping_rviz.rviz`
  - Rimosso RobotModel.
- `worlds/second_project.world`
  - Floorplan da `map/map.png`.
  - Robot `drive "omni"`.
  - Laser `ranger`.
  - Dimensione mappa aggiornata.
- `package.xml`
  - Dipendenze Nav2 esplicite.
  - Dipendenza runtime `stage_ros2_stageros`.
- `src/simple_robot_simulator.cpp`
  - Rimosso: era un simulatore interno fuori consegna.

## Cose ancora da fare

1. Installare davvero `stage_ros2_stageros` nella VM/container.
2. Compilare:

```bash
cd ~/colcon_ws
colcon build --packages-select stage_ros2_stageros second_project
source install/setup.bash
```

3. Rifare mapping con bag.
4. Salvare:
   - `map/map.png`
   - `map/map.yaml`
   - posegraph serialized di `slam_toolbox`
5. Testare navigation con Stage:

```bash
ros2 launch second_project navigation.launch.py
```

6. Creare `info.txt` per la consegna:

```text
codice_persona;nome;cognome
```

7. Creare eventualmente `readme.txt` spiegando che `stage_ros2_stageros` e incluso come dipendenza Stage ROS2 usata dai codici del corso.
8. Preparare tar finale.

## Consegna consigliata

Struttura tar consigliata:

```text
codicepersona.tar.gz
├── info.txt
├── readme.txt
└── src/
    ├── second_project/
    └── stage_ros2_stageros/
```

Non includere:

```text
build/
install/
log/
bags/
Code by Claudio/
Note by Claudio/
stage_ros2/        # TUW, non usato
*.zip
.DS_Store
```

## Verifiche statiche gia fatte

- `python3 -m py_compile launch/mapping.launch.py launch/navigation.launch.py`: OK.
- `package.xml` parse XML: OK.
- `git diff --check`: OK.
- Nessun path assoluto trovato nei file principali del progetto.

Nota: non e stato possibile fare `colcon build` dal Mac; la build vera va fatta nella VM/container ROS2.
