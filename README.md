# C5 P24 Eindopdracht — Visiongestuurde pick-and-place met ROS2

Deze repository bevat het ROS2-systeem voor de C5 P24 eindopdracht. Het project combineert objectdetectie, 3D-positiebepaling, robotaansturing en een gebruikersinterface tot één geïntegreerd systeem.

Het systeem gebruikt een Luxonis OAK-D camera voor vision, een YOLO-model voor objectdetectie, ArUco-markerreferentie voor world/robot-calibratie en een UFactory xArm Lite6 manipulator voor het oppakken van objecten.

Repository:

```text
https://github.com/Nibb2020/c5_p24_eindopdracht
```

---

## Inhoud

* [Projectdoel](#projectdoel)
* [Systeemoverzicht](#systeemoverzicht)
* [Repositorystructuur](#repositorystructuur)
* [Hardware](#hardware)
* [Softwareomgeving](#softwareomgeving)
* [Belangrijke ROS2-packages](#belangrijke-ros2-packages)
* [ROS2-interfaces](#ros2-interfaces)
* [Vision-systeem](#vision-systeem)
* [Robot- en manipulatiesysteem](#robot--en-manipulatiesysteem)
* [User interface](#user-interface)
* [Installatie](#installatie)
* [Python virtual environment](#python-virtual-environment)
* [Build-instructies](#build-instructies)
* [Volledig systeem starten](#volledig-systeem-starten)
* [Losse subsystemen starten](#losse-subsystemen-starten)
* [Belangrijke configuratie](#belangrijke-configuratie)
* [Dataset en modelstructuur](#dataset-en-modelstructuur)
* [Controlecommando's](#controlecommandos)
* [Veelvoorkomende fouten en oplossingen](#veelvoorkomende-fouten-en-oplossingen)
* [Bekende beperkingen](#bekende-beperkingen)

---

## Projectdoel

Het doel van dit project is om een robotarm autonoom objecten te laten herkennen, lokaliseren en oppakken.

De globale werking is:

1. De OAK-D camera maakt RGB- en depthbeelden.
2. Het vision-systeem detecteert objecten met een YOLO-model.
3. De 3D-positie van het object wordt bepaald met depthdata.
4. Een vaste ArUco-marker wordt gebruikt als referentie tussen camera-frame en robot/world-frame.
5. De objectpositie en yaw worden gepubliceerd via ROS2-interfaces.
6. De controller/state machine vraagt objectdata op.
7. De manipulator gebruikt deze data om de xArm Lite6 naar het object te bewegen.
8. De HMI toont/bedient de systeemstatus.

---

## Systeemoverzicht

Het systeem bestaat uit vier hoofdlagen:

```text
User Interface
    ↓
Controller / State Machine
    ↓
Vision + Manipulatie
    ↓
Hardware: OAK-D camera + xArm Lite6 robot
```

De belangrijkste datastroom is:

```text
OAK-D camera
    ↓
vision_node
    ↓
project_interfaces/ObjectData
    ↓
controller/state_machine
    ↓
manipulatie/manipulator
    ↓
xArm Lite6
```

---

## Repositorystructuur

De repository bevat onder andere de volgende ROS2-packages:

```text
c5_p24_eindopdracht/
├── controller/
├── manipulatie/
├── my_moveit_python/
├── my_ufactory_ROS2/
├── project_interfaces/
├── pymoveit2/
├── user_interface/
├── vision/
└── README.md
```

Belangrijkste mappen:

| Map                  | Functie                                                                            |
| -------------------- | ---------------------------------------------------------------------------------- |
| `controller`         | Supervisor, state machine en centrale systeemlogica                                |
| `vision`             | OAK-D camera, YOLO-detectie, depthverwerking, ArUco-calibratie en objectpublicatie |
| `manipulatie`        | Manipulatorlogica voor de xArm Lite6                                               |
| `user_interface`     | HMI/bedieningsinterface                                                            |
| `project_interfaces` | Custom ROS2 messages en services                                                   |
| `my_ufactory_ROS2`   | UFactory/xArm-gerelateerde ROS2-integratie                                         |
| `pymoveit2`          | MoveIt2 Python-aansturing                                                          |
| `my_moveit_python`   | MoveIt-gerelateerde hulppakketten                                                  |

---

## Hardware

Gebruikte hardware:

| Hardware                | Functie                             |
| ----------------------- | ----------------------------------- |
| Luxonis OAK-D camera    | RGB, stereo depth en objectdetectie |
| UFactory xArm Lite6     | Robotmanipulator                    |
| ArUco-marker            | World/robot-referentie voor vision  |
| Ubuntu ROS2-machine/VM  | Draait ROS2 Jazzy                   |
| Windows-host met VMware | Ontwikkelomgeving voor Ubuntu VM    |

Belangrijke aandachtspunten:

* De OAK-D moet via USB aan de Ubuntu VM gekoppeld zijn.
* De xArm Lite6 moet via netwerk bereikbaar zijn.
* De robot-IP staat in de launch/config ingesteld op:

```text
192.168.1.156
```

---

## Softwareomgeving

Geteste omgeving:

| Component | Versie / Opmerking                            |
| --------- | --------------------------------------------- |
| Ubuntu    | Ubuntu met ROS2 Jazzy                         |
| ROS2      | Jazzy                                         |
| Python    | 3.12                                          |
| DepthAI   | 2.29.0.0                                      |
| OpenCV    | `opencv-contrib-python` aanbevolen voor ArUco |
| NumPy     | 1.26.x                                        |
| MoveIt2   | ROS2 Jazzy MoveIt stack                       |
| xArm SDK  | 1.17.3 in logs                                |

De vision-node draait vanuit een Python virtual environment:

```text
~/c5_p24_eindproject_ws/.venv
```

---

## Belangrijke ROS2-packages

### `project_interfaces`

Bevat custom ROS2-interfaces zoals:

```text
srv/
├── GetObjectData.srv
├── Manipulator.srv
└── VoorwerpData.srv
```

Deze package moet altijd correct gebouwd zijn voordat `vision`, `controller`, `user_interface` en `manipulatie` gestart worden.

Controle:

```bash
python3 -c "from project_interfaces.srv import GetObjectData; print('project_interfaces OK')"
```

---

### `vision`

Bevat de vision-node. Deze node:

* maakt verbinding met de OAK-D;
* start een DepthAI pipeline;
* gebruikt YOLO voor objectdetectie;
* gebruikt stereo depth voor 3D-positie;
* gebruikt ArUco voor world/robot-calibratie;
* publiceert objectdata naar ROS2;
* biedt een service aan voor objectaanvragen.

---

### `controller`

Bevat onder andere:

* `system_supervisor`
* `state_machine`
* `main_system.launch.py`
* `full_system.launch.py`

De controller start de systeemlogica en kan ook de robot/manipulatorlaunch starten.

---

### `manipulatie`

Bevat de manipulatornode die de xArm Lite6 aanstuurt via MoveIt2/xArm-integratie.

---

### `user_interface`

Bevat de HMI-node.

De HMI kan proberen een lokale camera via `/dev/video0` te openen. Als er geen gewone webcam gekoppeld is, kan deze warning verschijnen:

```text
can't open camera by index
Camera index out of range
```

Dit is niet hetzelfde als de OAK-D-camera via DepthAI.

---

## ROS2-interfaces

### `ObjectData`

Globale structuur:

```text
string object_class
string object_id
float32 confidence
geometry_msgs/TransformStamped transform
```

Belangrijke velden:

```text
object_class
object_id
confidence
transform.transform.translation.x
transform.transform.translation.y
transform.transform.translation.z
transform.transform.rotation.z
```

Let op: in dit project wordt `rotation.z` gebruikt als pure yawwaarde in radialen. Dit is project-specifiek en geen standaard quaterniongebruik.

---

### `ObjectDataArray`

Globale structuur:

```text
project_interfaces/ObjectData[] objects
```

Wordt gebruikt om meerdere objecten richting UI te publiceren.

---

### `GetObjectData.srv`

Wordt gebruikt om vanuit de controller/state machine een objectmeting op te vragen bij vision.

Globale structuur:

```text
float32 confidence_threshold
---
bool success
project_interfaces/ObjectData object
```

Voorbeeld service-call:

```bash
ros2 service call /vision/voorwerp_data project_interfaces/srv/GetObjectData "{confidence_threshold: 0.85}"
```

---

## Vision-systeem

De vision-node voert de volgende stappen uit:

1. Verbinden met de OAK-D camera.
2. Aanmaken van de DepthAI pipeline.
3. Ophalen van RGB- en depthframes.
4. Objectdetectie met YOLO.
5. Bepalen van objectpositie met depth-ROI.
6. Bepalen van objectyaw met beeldverwerking/PCA.
7. Detecteren van een vaste ArUco-marker.
8. Transformeren van camera-frame naar robot/world-frame.
9. Publiceren van objectdata en gemarkeerd debugbeeld.
10. Beantwoorden van objectaanvragen via een ROS2-service.

Belangrijke topics/services:

| Naam                         | Type    | Functie                           |
| ---------------------------- | ------- | --------------------------------- |
| `/vision/voorwerp_data`      | Service | Objectdata opvragen               |
| `/vision/object_data_ui`     | Topic   | Objectarray richting UI           |
| `/vision/object_data_result` | Topic   | Beste object richting Lite6/debug |
| `/vision/marked_foto`        | Topic   | Gemarkeerd debugbeeld             |

---

## Robot- en manipulatiesysteem

De robotlaag gebruikt de xArm Lite6 en MoveIt2. De manipulator ontvangt objectdata vanuit de controller/state machine en stuurt de robot naar een doelpose.

De robotlaunch start onder andere:

* `robot_state_publisher`
* `move_group`
* `ros2_control_node`
* `xarm_driver_node`
* controller spawners
* RViz2
* statische transforms
* manipulatornode

Als de robot niet bereikbaar is, zijn deze fouten verwacht:

```text
Error: Tcp control connection failed
Segmentation fault
Could not contact service /controller_manager/list_controllers
```

Dit betekent niet automatisch dat de ROS2-packages fout zijn. Het betekent meestal dat de Lite6 niet bereikbaar is op het ingestelde IP-adres.

---

## User interface

De HMI wordt gestart via:

```text
user_interface/hmi_interface_versie12
```

De HMI publiceert onder andere naar:

```text
/ui/start_stop
```

Als er geen lokale webcam beschikbaar is, kunnen deze warnings verschijnen:

```text
VIDEOIO(V4L2:/dev/video0): can't open camera by index
Camera index out of range
```

Deze warnings zijn niet kritisch als de HMI geen gewone webcam nodig heeft.

---

## Installatie

### 1. Workspace aanmaken

```bash
mkdir -p ~/c5_p24_eindproject_ws/src
cd ~/c5_p24_eindproject_ws/src
```

### 2. Repository clonen

```bash
git clone https://github.com/Nibb2020/c5_p24_eindopdracht.git
```

De structuur wordt dan:

```text
~/c5_p24_eindproject_ws/src/c5_p24_eindopdracht
```

---

## Python virtual environment

Maak de virtual environment aan in de root van de workspace:

```bash
cd ~/c5_p24_eindproject_ws
python3 -m venv .venv
source .venv/bin/activate
```

Upgrade pip-tools:

```bash
python3 -m pip install --upgrade pip setuptools wheel
```

Installeer de belangrijkste Python dependencies:

```bash
python3 -m pip install depthai==2.29.0.0
python3 -m pip install opencv-contrib-python numpy
python3 -m pip install pillow transforms3d lark empy==3.3.4
```

Controleer de belangrijkste dependencies:

```bash
python3 -c "import depthai as dai; print(dai.__version__)"
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "from PIL import Image, ImageTk; print('PIL OK')"
python3 -c "import transforms3d; print('transforms3d OK')"
python3 -c "import lark; print('lark OK')"
python3 -c "import em; print(getattr(em, '__version__', 'no version'))"
```

Verwachte DepthAI-versie:

```text
2.29.0.0
```

Verwachte EmPy-versie:

```text
3.3.4
```

---

## Build-instructies

### Basis build

Gebruik altijd eerst ROS2 Jazzy en activeer daarna de `.venv`:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
```

Build eerst `project_interfaces` als die nog niet gebouwd is:

```bash
python3 -m colcon build --packages-select project_interfaces --symlink-install --allow-overriding project_interfaces
source install/setup.bash
```

Controleer:

```bash
python3 -c "from project_interfaces.srv import GetObjectData; print('project_interfaces OK')"
```

Build daarna de rest:

```bash
python3 -m colcon build --packages-select vision controller user_interface manipulatie --symlink-install --allow-overriding project_interfaces
source install/setup.bash
```

---

## Volledig systeem starten

Gebruik dit volledige commando om het complete systeem te builden en te starten:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --packages-select project_interfaces vision controller user_interface manipulatie --symlink-install --allow-overriding project_interfaces

source install/setup.bash
ros2 launch controller full_system.launch.py
```

Als alles al gebouwd is, volstaat meestal:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

ros2 launch controller full_system.launch.py
```

---

## Losse subsystemen starten

### Alleen vision starten

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

ros2 launch vision vision.launch.py
```

### Alleen controller/HMI starten

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

ros2 launch controller main_system.launch.py
```

### Vision image bekijken

```bash
ros2 topic list | grep image
ros2 run rqt_image_view rqt_image_view
```

Of direct:

```bash
ros2 run rqt_image_view rqt_image_view /vision/marked_foto
```

---

## Belangrijke configuratie

De visionconfiguratie staat in:

```text
vision/config/config.py
```

Belangrijke instellingen:

| Configwaarde                | Functie                                        |
| --------------------------- | ---------------------------------------------- |
| `DEBUG_PUBLISH_CONTINUOUS`  | Continu gemarkeerd beeld publiceren            |
| `DEBUG_DRAW_ARUCO_LIVE`     | ArUco-overlay tekenen in debugbeeld            |
| `RGB_WIDTH` / `RGB_HEIGHT`  | Resolutie voor RGB/NN-frame                    |
| `USB_SPEED`                 | USB-snelheid voor OAK-D                        |
| `USE_DEVICE_CALIBRATION`    | OAK-D EEPROM-calibratie gebruiken              |
| `SERVICE_NAME`              | Naam van vision-service                        |
| `UI_TOPIC`                  | Topic richting UI                              |
| `MARKED_IMAGE_TOPIC`        | Gemarkeerd beeldtopic                          |
| `LITE6_RESULT_TOPIC`        | Objectdata richting manipulator/debug          |
| `OBJECT_SAMPLE_COUNT`       | Aantal samples per stabiele objectmeting       |
| `OBJECT_SAMPLE_TIMEOUT_SEC` | Maximale meettijd                              |
| `MODEL_VERSION`             | Actieve modelversie                            |
| `ARUCO_MARKER_ID`           | ID van vaste ArUco-marker                      |
| `ARUCO_SIZE_M`              | Fysieke markermaat in meters                   |
| `ARUCO_WORLD_X/Y/Z`         | Markerpositie in robot/world-frame             |
| `ARUCO_TO_ROBOT_ROTATION`   | Rotatiematrix van ArUco-frame naar robot-frame |
| `ROBOT_FILTER_ENABLED`      | Objectfiltering voor robotbereik               |
| `ROBOT_MIN_X_M` etc.        | Robotbereikgrenzen                             |

---

## Dataset en modelstructuur

De vision-package gebruikt bij voorkeur paden relatief aan de source-package zelf.

Aanbevolen structuur:

```text
vision/
├── config/
│   └── config.py
├── dataset/
└── models/
    └── V0.5/
        └── best_openvino_2022.1_3shave.blob
```

De config kan automatisch het `.blob` model vinden binnen de opgegeven modelversiemap.

Voorbeeld:

```python
MODEL_VERSION = "V0.5"
```

Dan zoekt het systeem automatisch in:

```text
vision/models/V0.5/
```

Als daar precies één `.blob` bestand staat, wordt dat model gebruikt.

Controleer het gevonden modelpad met:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

python3 -c "from config.config import VISION_PACKAGE_DIR, DATASET_FOLDER, MODEL_PATH; print(VISION_PACKAGE_DIR); print(DATASET_FOLDER); print(MODEL_PATH)"
```

Verwachte output is vergelijkbaar met:

```text
/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision
/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/dataset
/home/student/c5_p24_eindproject_ws/src/c5_p24_eindopdracht/vision/models/V0.5/<modelnaam>.blob
```

---

## Controlecommando's

### ROS2 packages controleren

```bash
ros2 pkg list | grep project_interfaces
ros2 pkg list | grep vision
ros2 pkg list | grep controller
ros2 pkg list | grep user_interface
ros2 pkg list | grep manipulatie
```

### Topics bekijken

```bash
ros2 topic list
```

Visiontopics filteren:

```bash
ros2 topic list | grep vision
```

### Services bekijken

```bash
ros2 service list
```

Visionservice testen:

```bash
ros2 service call /vision/voorwerp_data project_interfaces/srv/GetObjectData "{confidence_threshold: 0.85}"
```

### TF controleren

```bash
ros2 run tf2_ros tf2_echo world link_base
ros2 run tf2_ros tf2_echo world xarm_link
ros2 run tf2_ros tf2_echo xarm_link link_base
```

### DepthAI controleren

```bash
python3 -c "import depthai as dai; print(dai.__version__); print(dai.Device.getAllAvailableDevices())"
```

### USB-apparaten controleren

```bash
lsusb
```

De OAK-D verschijnt meestal als:

```text
Intel Movidius MyriadX
```

---

## Veelvoorkomende fouten en oplossingen

### 1. `ModuleNotFoundError: No module named 'project_interfaces'`

Oorzaak:

* `project_interfaces` is niet gebouwd;
* `install/setup.bash` is niet gesourced;
* build is eerder gefaald.

Oplossing:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --packages-select project_interfaces --symlink-install --allow-overriding project_interfaces
source install/setup.bash

python3 -c "from project_interfaces.srv import GetObjectData; print('project_interfaces OK')"
```

---

### 2. `ModuleNotFoundError: No module named 'lark'`

Oorzaak:

ROS2 interfacegeneratie mist de Python package `lark`.

Oplossing:

```bash
source ~/c5_p24_eindproject_ws/.venv/bin/activate
python3 -m pip install lark
```

---

### 3. `em.TransientParseError: not enough data to read`

Oorzaak:

Verkeerde of incompatibele EmPy-versie in de `.venv`.

Oplossing:

```bash
source ~/c5_p24_eindproject_ws/.venv/bin/activate
python3 -m pip uninstall -y empy em
python3 -m pip install empy==3.3.4
```

Daarna `project_interfaces` schoon rebuilden:

```bash
cd ~/c5_p24_eindproject_ws
rm -rf build/project_interfaces install/project_interfaces

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --packages-select project_interfaces --symlink-install --allow-overriding project_interfaces
source install/setup.bash
```

---

### 4. `ModuleNotFoundError: No module named 'PIL'`

Oorzaak:

De HMI gebruikt Pillow, maar dit staat niet in de `.venv`.

Oplossing:

```bash
source ~/c5_p24_eindproject_ws/.venv/bin/activate
python3 -m pip install pillow
```

Controle:

```bash
python3 -c "from PIL import Image, ImageTk; print('PIL OK')"
```

---

### 5. `ModuleNotFoundError: No module named 'transforms3d'`

Oorzaak:

`tf_transformations` gebruikt intern `transforms3d`.

Oplossing:

```bash
source ~/c5_p24_eindproject_ws/.venv/bin/activate
python3 -m pip install transforms3d
```

Controle:

```bash
python3 -c "import transforms3d; import tf_transformations; print('transforms3d OK')"
```

---

### 6. OAK-D: `Connection failed: No available devices`

Oorzaak:

* OAK-D is niet aangesloten;
* OAK-D is niet gekoppeld aan de Ubuntu VM;
* USB-device hangt nog aan Windows host;
* verkeerde USB-reset/status.

Oplossing:

1. Controleer VMware USB-koppeling.
2. Koppel de OAK-D aan de Ubuntu VM.
3. Controleer:

```bash
lsusb
```

4. Controleer DepthAI:

```bash
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"
```

Als de camera bewust niet beschikbaar is, is deze melding normaal. De vision-node blijft reconnecten.

---

### 7. HMI: `/dev/video0` niet beschikbaar

Melding:

```text
VIDEOIO(V4L2:/dev/video0): can't open camera by index
Camera index out of range
```

Oorzaak:

De HMI probeert een gewone Linux webcam te openen. Dit staat los van de OAK-D.

Als de HMI geen lokale webcam nodig heeft, kan deze melding genegeerd worden.

---

### 8. xArm: `Tcp control connection failed`

Melding:

```text
robot_ip=192.168.1.156
Error: Tcp control connection failed
```

Oorzaak:

De Lite6 is niet bereikbaar op het ingestelde IP-adres.

Controleer:

```bash
ping 192.168.1.156
```

Als de robot niet aangesloten/bereikbaar is, zijn ook deze vervolgmeldingen verwacht:

```text
Segmentation fault
Could not contact service /controller_manager/list_controllers
```

---

### 9. `AMENT_PREFIX_PATH` warning over oude package

Voorbeeld:

```text
The path '/home/student/c5_p24_eindproject_ws/install/manipulation' in the environment variable AMENT_PREFIX_PATH doesn't exist
```

Oorzaak:

Er staat nog een oude package- of workspaceverwijzing in de terminalomgeving.

Oplossing:

Open een nieuwe terminal en source alleen de benodigde setupfiles:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Als het blijft terugkomen, kan een schone build helpen:

```bash
cd ~/c5_p24_eindproject_ws
rm -rf build install log

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --symlink-install --allow-overriding project_interfaces
source install/setup.bash
```

---

## Bekende beperkingen

* De vision-node heeft een aangesloten OAK-D nodig voor echte objectdetectie.
* De robotlaunch probeert de xArm Lite6 te bereiken op `192.168.1.156`.
* Zonder robotverbinding kunnen xArm-driver en `ros2_control_node` crashen.
* Zonder `/dev/video0` kan de HMI webcamwarnings geven.
* `rotation.z` in `ObjectData` wordt project-specifiek gebruikt als yaw in radialen, niet als standaard quaternioncomponent.
* Het `.blob` model moet aanwezig zijn in de ingestelde modelversiemap onder `vision/models/<MODEL_VERSION>/`.

---

## Aanbevolen normale workflow

Bij een normale werkdag:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch controller full_system.launch.py
```

Na codewijzigingen:

```bash
cd ~/c5_p24_eindproject_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --packages-select project_interfaces vision controller user_interface manipulatie --symlink-install --allow-overriding project_interfaces

source install/setup.bash
ros2 launch controller full_system.launch.py
```

---

## Projectstatus

Het geïntegreerde ROS2-systeem kan softwarematig starten vanuit de vision `.venv`. De correcte werking van de volledige keten vereist dat zowel de OAK-D camera als de xArm Lite6 robot beschikbaar zijn.

Wanneer camera en robot niet beschikbaar zijn, zijn de volgende meldingen verwacht:

```text
Connection failed: No available devices
Tcp control connection failed
Could not contact service /controller_manager/list_controllers
```

Deze meldingen betekenen in dat geval niet dat de build of launchfile fout is.
