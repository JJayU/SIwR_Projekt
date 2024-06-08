# Projekt SIwR

Jakub Junkiert 2024

## Cel projektu

Projekt polega na stworzeniu systemu estymacji stanu robota wykorzystującego probablistyczne modele grafowe. System ma za zadanie lokalizować robota dwukołowego w znanej mapie na podstawie informacji pochodzącej z wielu czujników.

## Opis działania

TODO

## Uruchamianie

Pakiet należy zbudować:
```bash
colcon build
. install/setup.bash
```

Pierwszym krokiem jest uruchomienie symulacji z wykorzystaniem następujących komend:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Następnie należy uruchomić Rviz2:
```bash
rviz2 -d /home/rosdev/ros2_ws/src/projekt_siwr/config/rviz_config.rviz
```

Ostatnim krokiem jest uruchomienie paczki z projektem poprzez komendę:
```bash
ros2 run projekt_siwr projekt_siwr
```

W celu poruszania robotem można uruchomić program:
```bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

TODO zmiana pliku spawn

pip install utm