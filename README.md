# `Lidar2D`

## 1. Introdução

Este repositório apresenta um projeto de fusão de dados do LiDAR 2D com informações de uma câmera, visando integrar os objetos reconhecidos pelo LiDAR às classificações obtidas pela câmera. O objetivo é aprimorar a detecção e identificação de objetos em ambientes complexos, combinando as vantagens de ambas as tecnologias.

## 2. Estrutura de Pastas

A estrutura do repositório deve seguir o seguinte formato:

lidar2d/ 
│
│
├── src/
│
│   ├── lidar_pkg/
│
│   │   ├── include/
│
│   │   ├── msg/
│
│   │   │   └── Objeto.msg
│
│   │   ├── scripts/
│
│   │   │   ├── biblioteca_lidar_camera.py
│
│   │   │   ├── Lidar_camera_objetos.py
│
│   │   │   ├── configuracao.cfg
│
│   │   │   ├── configuracao.md
│
│   │   │   ├── __init__.py
│
│   │   │   └── objetos_bb.cfg
│
│   │   ├── CMakeLists.txt
│
│   │   └── package.xml
│
│   ├── libsick_ldmrs/
│
│   └── sick_scan_xd/
│
└── README.md
