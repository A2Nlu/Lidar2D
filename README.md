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

## 3. lidar_pkg
  O pacote lidar_pkg é um package feito para fundir as informações do lidar 2d com os da camera. os testes foram realizados utilizando ROS2 Iron.
  ### 1. Instalação 
  ```bash
  git clone https://github.com/A2Nlu/Lidar2D.git
  ```
  No arquivo CMakeLists.txt comente a linha `find_package(lidar_pkg REQUIRED)`,   pois para o primeiro build do pacote essa linha da erro na compilação do pkg. Use o comando:
    
  ```bash
  cd lidar2d
  colcon build
  ```
  para a compilação do pacote, apos o sucesso da compilação descomente a linha `find_package(lidar_pkg REQUIRED)` do arquivo CMakeLists.txt e repita a compilação.

  ![linha a se comentar para o primeiro build](imagens/comentario.jpg)

  ## 2. Pacote da SICK
  Para a comunicação com o LIDAR2D e recebimento dos dados, foi utilizado o pacote disponibilizado pela SICK. Para mais informações sobre o pacote acessar o reporsitorio da [SICK](https://github.com/SICKAG/sick_scan_xd?tab=readme-ov-file), ou seguir os seguintes passos:

```bash
cd lidar2d/src
git clone https://github.com/SICKAG/libsick_ldmrs.git
git clone -b master https://github.com/SICKAG/sick_scan_xd.git
popd
rm -rf ./build ./build_isolated/ ./devel ./devel_isolated/ ./install ./install_isolated/ ./log/
cd ..
source /opt/ros/iron/setup.bash # mude a versão do ROS2 conforme necessário
colcon build --packages-select libsick_ldmrs --event-handlers console_direct+
source ./install/setup.bash
colcon build --packages-select sick_scan_xd --cmake-args "-DROS_VERSION=2" --event-handlers console_direct+
source ./install/setup.bash
colcon build
```
OBS: faça o build de cada pacote um por vez, espere terminar para ter certeza de que não deu erro na anterior, colocar para fazer uma seguida da outra de uma vez da erro!

Para testar o pacote basta usar o comando: 
```bash
ros2 launch sick_scan_xd sick_lms_1xx.launch.py hostname:=192.168.1.64 
```
Apos iniciar, nos topicos do ros2 deve conter:

***** add uma imagem


