# `Lidar2D`

## 1. Introdução

Este repositório apresenta um projeto de fusão de dados do LiDAR 2D com informações de uma câmera, utilizando ROS2, visando integrar os objetos reconhecidos pelo LiDAR às classificações obtidas pela câmera. O objetivo é aprimorar a detecção e identificação de objetos em ambientes complexos, combinando as vantagens de ambas as tecnologias.

## 2. Estrutura de Pastas
Este Reporsitorio comtem somente o PKG da fusão dos dados do lidar e da camera. Os dados do lidar é proveniente de uma biblioteca da sick, e os da camera vem de um topico explicado em um outro reporsitorio. 
Sendo assim a estrutura a seguir é como deve ficar a workspace referente ao lidar2D.

lidar2d/

├── src/

│     ├── lidar_pkg/

│     │     ├── include/

│     │     ├── msg/

│     │     │     └── Objeto.msg

│     │     ├── scripts/

│     │     │     ├── biblioteca_lidar_camera.py

│     │     │     ├── Lidar_camera_objetos.py

│     │     │     ├── configuracao.cfg

│     │     │     ├── configuracao.md

│     │     │     ├── __init__.py

│     │     │     └── objetos_bb.cfg

│     │     ├── CMakeLists.txt

│     │     └── package.xml

│     ├── libsick_ldmrs/

│     └── sick_scan_xd/

└── README.md

## 3. Pacote da SICK
  Para a comunição com o LIDAR2D e recebimento dos dados, foi utilizado o pacote dispolibilizado pela SICK, para a sua instalação pode-se acessar: https://github.com/SICKAG/sick_scan_xd/blob/develop/INSTALL-ROS2.md#build-on-linux-ros2, ou seguiur os seguintes passos:
1.```
cd lidar2d/src
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
  ```
5. popd
6. rm -rf ./build ./build_isolated/ ./devel ./devel_isolated/ ./install ./install_isolated/ ./log/
7. cd ..
8. source /opt/ros/iron/setup.bash # mude a versão do ROS2 conforme necessario
9. colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
10. source ./install/setup.bash
11. colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+
12. source ./install/setup.bash
13. colcon build

  Para testar o pacote basta usar o comando: 
  # ros2 launch sick_scan_xd sick_lms_1xx.launch.py hostname:=192.168.1.64 

## 4. lidar_pkg
