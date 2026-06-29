# C5 P24 Eindopdracht

Deze repository bevat het ROS2-systeem voor de C5 P24 eindopdracht. Het project combineert objectdetectie, positiebepaling, robotaansturing, sorteercycluslogica en een gebruikersinterface tot één geïntegreerd systeem.

Het systeem gebruikt een Luxonis OAK-D camera voor vision, een YOLO-model voor objectdetectie, klassieke beeldverwerking voor het bepalen van een stabiel pickpunt, een ArUco-marker als world/robot-referentie en een UFactory xArm Lite6 manipulator voor het oppakken en sorteren van objecten.

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
* [Aanbevolen normale workflow](#aanbevolen-normale-workflow)
* [Projectstatus](#projectstatus)

---

## Projectdoel

Het doel van dit project is om een robotarm autonoom objecten te laten herkennen, lokaliseren, oppakken en sorteren.

De globale werking is:

1. De OAK-D camera maakt RGB-beelden en optioneel depthbeelden.
2. Het vision-systeem detecteert objecten met een YOLO-model.
3. De boundingbox van YOLO wordt gebruikt als zoekgebied voor klassieke vision.
4. Klassieke beeldverwerking bepaalt het optische zwaartepunt, de langste objectas en het uiteindelijke pickpunt.
5. De X/Y-positie wordt berekend vanuit het gekozen beeldpunt en een vaste of gemeten projectiediepte.
6. De Z-positie wordt per objectklasse vast ingesteld.
7. Een vaste ArUco-marker wordt gebruikt als referentie tussen camera-frame en robot/world-frame.
8. De objectpositie en yaw worden gepubliceerd via ROS2-interfaces.
9. De controller/state machine vraagt objectdata op.
10. De manipulator gebruikt deze data om de xArm Lite6 naar het object te bewegen.
11. De HMI toont de systeemstatus, bedient het systeem en telt succesvol gesorteerde objecten.

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

De tellerlogica in de UI gebruikt niet langer het verdwijnen van objecten uit het camerabeeld. De UI telt een object pas als de manipulator expliciet `Klaar` meldt na een succesvolle sorteercyclus.

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

| Map                  | Functie                                                                                       |
| -------------------- | --------------------------------------------------------------------------------------------- |
| `controller`         | Supervisor, state machine en centrale systeemlogica                                           |
| `vision`             | OAK-D camera, YOLO-detectie, klassieke pickpuntbepaling, ArUco-calibratie en objectpublicatie |
| `manipulatie`        | Manipulatorlogica voor de xArm Lite6                                                          |
| `user_interface`     | HMI/bedieningsinterface en sorteertellers                                                     |
| `project_interfaces` | Custom ROS2 messages en services                                                              |
| `my_ufactory_ROS2`   | UFactory/xArm-gerelateerde ROS2-integratie                                                    |
| `pymoveit2`          | MoveIt2 Python-aansturing                                                                     |
| `my_moveit_python`   | MoveIt-helperlogica, inclusief planning-retries en bewegingen naar pose/jointconfiguratie     |

---

## Hardware

Gebruikte hardware:

| Hardware                | Functie                             |
| ----------------------- | ----------------------------------- |
| Luxonis OAK-D camera    | RGB, stereo depth en objectdetectie |
| UFactory xArm Lite6     | Robotmanipulator                    |
| Vacuumgripper           | Oppakken van objecten               |
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
* gebruikt klassieke vision voor het pickpunt;
* gebruikt optioneel depth voor projectiediepte;
* gebruikt vaste Z-waarden per objectklasse;
* gebruikt ArUco voor world/robot-calibratie;
* publiceert objectdata naar ROS2;
* publiceert een gemarkeerd debugbeeld;
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

De manipulator:

* ontvangt objectklasse, positie en yaw vanuit de controller;
* beweegt eerst naar een veilige positie boven het object;
* beweegt daarna naar de pickpositie;
* schakelt de vacuumgripper;
* beweegt naar de juiste dropstaat;
* publiceert alleen `Klaar` als de volledige sorteercyclus succesvol is;
* publiceert een foutstatus als een beweging of planning mislukt.

---

### `my_moveit_python`

Bevat de MoveIt-helperklasse.

Belangrijke functies:

* `move_to_pose()`
* `move_to_configuration()`
* `compute_fk()`
* `compute_ik()`

De huidige versie ondersteunt planning-retries. Bij planning failure kan MoveIt maximaal 10 pogingen doen voordat de beweging als mislukt wordt beschouwd.

---

### `user_interface`

Bevat de HMI-node.

De HMI:

* publiceert start/stop naar de controller;
* toont robotstatus, waarschuwingen en errors;
* toont het gemarkeerde visionbeeld;
* stelt robotparameters in, zoals velocity scaling en acceleration scaling;
* stelt vision confidence in;
* biedt reset, retry en home-functionaliteit;
* telt gesorteerde objecten pas na een succesvolle manipulatorstatus `Klaar`.

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

Wordt gebruikt om objectdata richting UI te publiceren.

In de huidige systeemversie publiceert vision bij een objectrequest meestal het definitieve object dat naar de manipulator gaat. De UI gebruikt dit bericht om de klasse van het actieve sorteerobject tijdelijk op te slaan.

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

### `Manipulator.srv`

Wordt gebruikt om de manipulator te starten met een gedetecteerd object.

Globale structuur kan projectafhankelijk zijn, maar bevat in ieder geval:

```text
klasse
translation
rotation
---
succes
```

---

## Vision-systeem

De vision-node voert de volgende stappen uit:

1. Verbinden met de OAK-D camera.
2. Aanmaken van de DepthAI pipeline.
3. Ophalen van RGB-, depth- en YOLO-packets.
4. Objectdetectie met YOLO.
5. Bepalen van een cropgebied op basis van de YOLO-boundingbox.
6. Segmenteren van het object binnen de crop met klassieke vision.
7. Bepalen van het optische zwaartepunt.
8. Bepalen van de langste objectas met PCA.
9. Optioneel verschuiven van het pickpunt over de langste objectas richting de dikkere objectzijde.
10. Projecteren van het gekozen beeldpunt naar camera-X/Y.
11. Gebruiken van een vaste Z-waarde per objectklasse.
12. Detecteren van een vaste ArUco-marker.
13. Transformeren van camera-frame naar robot/world-frame.
14. Publiceren van objectdata en gemarkeerd debugbeeld.
15. Beantwoorden van objectaanvragen via een ROS2-service.

Belangrijke topics/services:

| Naam                         | Type    | Functie                           |
| ---------------------------- | ------- | --------------------------------- |
| `/vision/voorwerp_data`      | Service | Objectdata opvragen               |
| `/vision/object_data_ui`     | Topic   | Objectdata richting UI            |
| `/vision/object_data_result` | Topic   | Beste object richting Lite6/debug |
| `/vision/marked_foto`        | Topic   | Gemarkeerd debugbeeld             |

---

## Klassieke pickpuntbepaling

De pickpuntbepaling werkt als volgt:

```text
YOLO-boundingbox
    ↓
crop met marge
    ↓
GrabCut / voorgrondmasker
    ↓
grootste objectcomponent
    ↓
optisch zwaartepunt
    ↓
PCA-langste as
    ↓
optionele verschuiving over de langste as
    ↓
pick_center_x / pick_center_y
```

In het gemarkeerde beeld:

| Markering   | Betekenis                                             |
| ----------- | ----------------------------------------------------- |
| Paarse stip | Optisch zwaartepunt van het objectmasker              |
| Rode stip   | Uiteindelijke pickpositie                             |
| Rode lijn   | Verschuiving van optisch zwaartepunt naar pickpositie |
| Blauwe as   | Langste objectas                                      |
| Gele as     | Gripperas / haakse as                                 |

De verschuiving per klasse staat in `vision/config/config.py`:

```python
PICK_CENTER_SHIFT_PX = {
    0: 0.0,   # schip
    1: 0.0,   # dino
    2: 0.0,   # olifant
    3: 0.0,   # smiley
}
```

Positieve waarden verschuiven het pickpunt in de berekende positieve objectrichting. Negatieve waarden verschuiven in de tegenovergestelde richting.

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

De manipulator voert een sorteeractie uit in deze volgorde:

```text
1. Naar positie boven object
2. Naar objectpositie
3. Vacuumgripper sluiten
4. Terug omhoog
5. Naar dropstaat op basis van objectklasse
6. Vacuumgripper openen
7. Naar up/home-positie
8. Status "Klaar" publiceren
```

Bij een fout publiceert de manipulator geen `Klaar`, maar een foutstatus zoals:

```text
Fout: manipulator sequence mislukt
```

Dit is belangrijk voor de UI-teller.

---

## MoveIt-planning en retries

De MoveIt-helper kan bewegingen meerdere keren proberen te plannen.

Belangrijke parameters:

```text
max_planning_attempts
planning_retry_delay_sec
velocity_scaling
acceleration_scaling
```

Standaard:

```text
max_planning_attempts = 10
planning_retry_delay_sec = 0.2
velocity_scaling = 0.1
acceleration_scaling = 0.1
```

De retrylogica helpt vooral bij stochastische OMPL-planning. Bij echte structurele collisions, bijvoorbeeld tussen gripper/object en een obstacle, blijven meerdere pogingen waarschijnlijk falen.

Voorbeelden van structurele collisionmeldingen:

```text
Found a contact between 'held_object_link' and 'Tussenwand_Link'
Found a contact between 'gripper_lite6_link' and 'CamStatief_Link'
```

In zulke gevallen moeten de SRDF-states, collisionobjecten, tussenposities of drop-posities worden aangepast.

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

De HMI abonneert zich onder andere op:

```text
/controller/state
/controller/warning
/controller/error
/vision/object_data_ui
/vision/marked_foto
/manipulator/status
```

De sorteerteller werkt als volgt:

```text
vision publiceert objectdata
    ↓
UI onthoudt objectklasse als pending sorteerobject
    ↓
manipulator voert sorteercyclus uit
    ↓
manipulator/status == "Klaar"
    ↓
UI telt object +1
```

Bij manipulatorfouten telt de UI niets.

Voorbeelden:

```text
Klaar
```

betekent: object succesvol gesorteerd, teller +1.

```text
Fout: manipulator sequence mislukt
```

betekent: object niet tellen.

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

| Configwaarde                 | Functie                                        |
| ---------------------------- | ---------------------------------------------- |
| `DEBUG_PUBLISH_CONTINUOUS`   | Continu gemarkeerd beeld publiceren            |
| `DEBUG_DRAW_ARUCO_LIVE`      | ArUco-overlay tekenen in debugbeeld            |
| `RGB_WIDTH` / `RGB_HEIGHT`   | Resolutie voor RGB/NN-frame                    |
| `USB_SPEED`                  | USB-snelheid voor OAK-D                        |
| `USE_DEVICE_CALIBRATION`     | OAK-D EEPROM-calibratie gebruiken              |
| `SERVICE_NAME`               | Naam van vision-service                        |
| `UI_TOPIC`                   | Topic richting UI                              |
| `MARKED_IMAGE_TOPIC`         | Gemarkeerd beeldtopic                          |
| `LITE6_RESULT_TOPIC`         | Objectdata richting manipulator/debug          |
| `OBJECT_SAMPLE_COUNT`        | Aantal samples per stabiele objectmeting       |
| `OBJECT_SAMPLE_TIMEOUT_SEC`  | Maximale meettijd                              |
| `MODEL_VERSION`              | Actieve modelversie                            |
| `USE_CLASSICAL_PICK_CENTER`  | Klassieke vision gebruiken voor pickpunt       |
| `USE_FIXED_PROJECTION_DEPTH` | Vaste projectiediepte gebruiken voor X/Y       |
| `FIXED_PROJECTION_DEPTH_M`   | Camera-projectiediepte in meters               |
| `USE_FIXED_OBJECT_Z`         | Vaste Z per objectklasse gebruiken             |
| `FIXED_OBJECT_Z_M`           | Z-waarde per objectklasse                      |
| `PICK_CENTER_SHIFT_PX`       | Pickpuntverschuiving per klasse in pixels      |
| `ARUCO_MARKER_ID`            | ID van vaste ArUco-marker                      |
| `ARUCO_SIZE_M`               | Fysieke markermaat in meters                   |
| `ARUCO_WORLD_X/Y/Z`          | Markerpositie in robot/world-frame             |
| `ARUCO_TO_ROBOT_ROTATION`    | Rotatiematrix van ArUco-frame naar robot-frame |
| `ROBOT_FILTER_ENABLED`       | Objectfiltering voor robotbereik               |
| `ROBOT_MIN_X_M` etc.         | Robotbereikgrenzen                             |

Voorbeeld van relevante visionconfiguratie:

```python
USE_FIXED_OBJECT_Z = True
USE_CLASSICAL_PICK_CENTER = True
USE_FIXED_PROJECTION_DEPTH = True

FIXED_PROJECTION_DEPTH_M = 0.720

FIXED_OBJECT_Z_M = {
    0: 0.113,
    1: 0.102,
    2: 0.092,
    3: 0.0835,
}

PICK_CENTER_SHIFT_PX = {
    0: 0.0,
    1: 0.0,
    2: 0.0,
    3: 0.0,
}
```

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

Manipulatorstatus bekijken:

```bash
ros2 topic echo /manipulator/status
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

### 7. OAK-D: `Stereo alignment error`

Melding:

```text
[StereoDepth] [error] Stereo alignment error: 1, trying to recover.
```

Oorzaak:

De stereo-depth pipeline heeft tijdelijk problemen met alignment of synchronisatie.

Oplossing:

* Controleer USB-koppeling met de VM.
* Controleer of de OAK-D stabiel verbonden blijft.
* Gebruik voor X/Y-bepaling bij voorkeur `USE_FIXED_PROJECTION_DEPTH = True`, zodat instabiele depth minder invloed heeft op de pickpositie.

---

### 8. HMI: `/dev/video0` niet beschikbaar

Melding:

```text
VIDEOIO(V4L2:/dev/video0): can't open camera by index
Camera index out of range
```

Oorzaak:

De HMI probeert een gewone Linux webcam te openen. Dit staat los van de OAK-D.

Als de HMI geen lokale webcam nodig heeft, kan deze melding genegeerd worden.

---

### 9. xArm: `Tcp control connection failed`

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

### 10. MoveIt: `INVALID_MOTION_PLAN`

Melding:

```text
PlanningResponseAdapter 'ValidateSolution' failed with error code INVALID_MOTION_PLAN
Generating a plan with planning pipeline failed.
```

Oorzaak:

MoveIt heeft een pad gevonden dat achteraf ongeldig blijkt, vaak door collision of joint/trajectory-validatie.

Bijvoorbeeld:

```text
Found a contact between 'held_object_link' and 'Tussenwand_Link'
Found a contact between 'gripper_lite6_link' and 'CamStatief_Link'
```

Oplossingen:

* Laat MoveIt meerdere planningpogingen doen.
* Verplaats dropstates verder van obstakels.
* Voeg een veilige tussenpositie toe.
* Controleer collisionobjecten.
* Maak collisionmodellen van object/gripper realistischer.
* Verhoog de drop- of tussenpositie.

---

### 11. UI-teller telt verkeerd

Oude oorzaak:

De UI telde objecten die uit de visionlijst verdwenen. Dat is onbetrouwbaar, omdat objecten ook kunnen verdwijnen door beweging, confidence, filtering of camerabeeld.

Nieuwe werking:

```text
visionbericht → pending objectklasse opslaan
manipulator/status = Klaar → teller +1
manipulator/status = Fout... → niet tellen
```

Controle:

```bash
ros2 topic echo /manipulator/status
```

De UI telt alleen bij exact succesvolle sortering.

---

### 12. `AMENT_PREFIX_PATH` warning over oude package

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
* DepthAI-depth is niet betrouwbaar genoeg als enige basis voor het pickpunt; daarom gebruikt het systeem klassieke vision voor het X/Y-pickpunt.
* Vaste Z-waarden per objectklasse moeten handmatig worden getuned voor de gebruikte objecten.
* MoveIt-retries lossen geen structurele collisionproblemen op.
* De UI-teller is afhankelijk van correcte manipulatorstatussen. De manipulator mag alleen `Klaar` publiceren als de volledige sorteeractie echt is gelukt.

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

Bij twijfel of oude builds de oorzaak zijn:

```bash
cd ~/c5_p24_eindproject_ws
rm -rf build install log

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 -m colcon build --packages-select project_interfaces vision controller user_interface manipulatie --symlink-install --allow-overriding project_interfaces

source install/setup.bash
ros2 launch controller full_system.launch.py
```

---

## Projectstatus

Het geïntegreerde ROS2-systeem kan softwarematig starten vanuit de vision `.venv`. De correcte werking van de volledige keten vereist dat zowel de OAK-D camera als de xArm Lite6 robot beschikbaar zijn.

De huidige systeemversie bevat:

* YOLO-objectdetectie op de OAK-D;
* klassieke vision voor pickpuntbepaling;
* optisch zwaartepunt en PCA-objectas;
* vaste Z-waarden per objectklasse;
* optionele vaste projectiediepte voor stabielere X/Y-projectie;
* ArUco-gebaseerde camera-naar-robottransformatie;
* MoveIt-aansturing van de xArm Lite6;
* planning-retries bij MoveIt-planning failures;
* manipulatorstatussen die onderscheid maken tussen succes en fout;
* UI-tellers die alleen optellen na een succesvolle manipulatorstatus `Klaar`.

Wanneer camera en robot niet beschikbaar zijn, zijn de volgende meldingen verwacht:

```text
Connection failed: No available devices
Tcp control connection failed
Could not contact service /controller_manager/list_controllers
```

Deze meldingen betekenen in dat geval niet dat de build of launchfile fout is.

