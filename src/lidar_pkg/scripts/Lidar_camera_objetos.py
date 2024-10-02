#!/usr/bin/env python3

#ano 2024
#codigo com biblioteca inicio 30/08

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import LaserScan
from lidar_pkg.msg import Objeto
import pandas as pd
from vision_msgs.msg import Detection2DArray
import time
import setproctitle
import can
from scripts.biblioteca_lidar_camera import PontosCartezianos, PontosPolares, Agrupamento, Localizacao, localizacao_objeto,localizacaoObjetoCamera, FusaoLidarCamera, tipo_camera, tipo_objeto, format_data, send_can_message

setproctitle.setproctitle("Lidar_Camera_obj")

class DistanceMinFinder(Node):
    #def que inicia o nó, se inscreve e publica as mensagens, inicializa tudo
    def __init__(self):
        super().__init__('distance_min_finder')
        self.subscription_objetos = self.create_subscription(Float32MultiArray,'range_ang2', self.distance_callback, 10) #recebe os dados do lidar
        self.subscription_bounding_boxes = self.create_subscription(Detection2DArray, "agx/boundingBoxes", self.callback_bounding_boxes, 10) #recebe os dados da camera
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription_ranges = self.create_subscription(LaserScan,'/scan_filtered2', self.angulos_callback, 10)
        
        self.min_distance = float('inf')
        
        self.publisher_scan_filtered = self.create_publisher(LaserScan, '/scan_filtered2', 10)
        self.publisher_range_ang = self.create_publisher(Float32MultiArray, 'range_ang2', 10)
        self.publisher_objetos_lidar_camera = self.create_publisher(Objeto, 'objetos_lidar_camera2', 10) # topico de publicacao dos objetos
        self.aviso_publisher = self.create_publisher(String, 'aviso_lidar_camera2', 10) # topico de aviso caso a camera ou o lidar parem de enviar mensagem
        
        self.last_lidar_data_time = time.time()  # Inicializar o tempo do último dado do lidar
        self.last_camera_data_time = time.time()  # Inicializar o tempo do último dado da câmera
        self.lidar_data_timeout = 0.2 # Tempo limite em segundos para dados do lidar
        self.camera_data_timeout = 0.2  # Tempo limite em segundos para dados da câmera
        self.camera_results = [] #dados da camera
        self.pontos_ordenados_lidar = [] #dados do lidar
        self.can_bus = can.interface.Bus(channel='can0', interface='socketcan')
        
        self.get_logger().info('DistanceMinFinder node has started')
    
    #def para receber os dados do lidar
    def distance_callback(self, msg):
        self.last_lidar_data_time = time.time() #atualiza a ultima mensagem recebida 
        data = msg.data
        
        angulos = data[::2]
        distances = data[1::2]
        
        if data:
            # Ordenar os pontos pelo ângulo
            pontos_ordenados = sorted(zip(distances, angulos), key=lambda x: x[1])
        
            self.pontos_ordenados_lidar = pontos_ordenados
            
        else:
            pontos_ordenados = []
            self.pontos_ordenados_lidar = []
            
        self.ProgramaPrincipal()
    
    #def que relaciona os agulos as suas respectivas distancias
    def angulos_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Lista para armazenar os ângulos e alcances
        data = []

        # Calculando os ângulos correspondentes
        current_angle = angle_min
        for i in range(len(msg.ranges)):
            angle = current_angle
            current_angle += angle_increment
            range_value = msg.ranges[i]
            data.append(angle)
            data.append(range_value)

        # Publicando os dados no novo tópico
        msg = Float32MultiArray()
        msg.data = data
        self.publisher_range_ang.publish(msg)
    
    #def que filtra as distancias
    def scan_callback(self, msg):
        
        dados = pd.read_csv('/home/fatec/ros2_ws/src/lidar_pkg/scripts/configuracao.cfg')
        
        distancia_min = float(dados.iloc[0,1])
        distancia_max = float(dados.iloc[1,1])
        
        if distancia_min < 0.1:
            distancia_min = 0.1
        
        if distancia_max > 40:
            distancia_max = 40
        
        filtered_ranges = [r if distancia_min < r < distancia_max else float('0') for r in msg.ranges]
        msg.ranges = filtered_ranges
        self.publisher_scan_filtered.publish(msg)
              
    #def para receber os dados da camera
    def callback_bounding_boxes (self, msg): 
        self.last_camera_data_time = time.time() #atualiza a ultima mensagem recebida
        
        resultados = []
        centro_x = 0.0
        centro_y = 0.0
        angulo = 0.0
        tipo = ''
        
        for detection in msg.detections:
                
            centro_x = detection.bbox.center.x
            centro_y = detection.bbox.center.y
            angulo = detection.bbox.center.theta
            tipo = detection.results[0].id
            resultados.append((centro_x, centro_y, angulo, tipo))
        
        if resultados:
            self.camera_results = resultados
        else:
            resultados = []
            self.camera_results = []
            
        self.ProgramaPrincipal()
    
    #Programa principal, que chama as def   
    def ProgramaPrincipal(self):
        current_time = time.time()
        
        # Verificar se os dados do lidar estão atualizados
        if current_time - self.last_lidar_data_time > self.lidar_data_timeout:
            #self.get_logger().warn('Falha ao receber dados do lidar. Limpando dados do lidar.')
            dados_lidar = []  # Esvaziar os dados do lidar
            self.publish_aviso("Falha ao receber dados do lidar.")
        else:
            dados_lidar = self.pontos_ordenados_lidar
        
        # Verificar se os dados da câmera estão atualizados
        if current_time - self.last_camera_data_time > self.camera_data_timeout:
            #self.get_logger().warn('Falha ao receber dados da câmera. Limpando dados da câmera.')
            dados_camera = []  # Esvaziar os dados da câmera
            self.publish_aviso("Falha ao receber dados da câmera.")
            
        else:
            dados_camera = self.camera_results
        
        pontos_cartezianos = PontosCartezianos(dados_lidar) 
        
        objetos = Agrupamento(pontos_cartezianos)
        
        #self.plotar_pontos(objetos)
        
        pontos_objeto = PontosPolares(objetos)
        
        geografia_obj = Localizacao(pontos_objeto)
        
        camera_objeto = localizacaoObjetoCamera(dados_camera)
        
        objeto_final = FusaoLidarCamera(geografia_obj, camera_objeto)
        
        self.print_object_info(objeto_final)

        self.mensagemCan(objeto_final)
           
    def mensagemCan(self, objeto):
        #can_channel = 'can0'  # Canal do PCAN 
        msg_id = 0x18FF0003 # ID da mensagem

        idx = 0

        for idx, obj in enumerate(objeto, 1):
            distancia, angulo, tamanho, risco, localizacao, tipo = obj
            localizacao_cod = localizacao_objeto(localizacao)
            tipo_obj = tipo_camera(tipo)
            tipo_cod = tipo_objeto(tipo_obj)
            
            dados = format_data(idx, localizacao_cod, distancia, angulo, tipo_cod, tamanho, risco)
            
            # Envia as mensagens 
            send_can_message(self.can_bus, msg_id, dados)

    #def para publicar os objetos
    def print_object_info(self, objects):
        idx = 0
        if objects:
            for idx, obj in enumerate(objects, 1):
                distancia_central, angulo_central, tamanho_do_objeto, risco,  localizacao, tipo = obj

                # Criar e preencher a mensagem com os dados do objeto
                msg = Objeto()
                msg.label = localizacao
                msg.id = idx
                msg.distancia_central = distancia_central
                msg.angulo_central = angulo_central
                msg.tamanho_do_objeto = tamanho_do_objeto
                msg.risco = risco
                msg.tipo = tipo
                
                # Publicar a mensagem no tópico /objetos
                self.publisher_objetos_lidar_camera.publish(msg)
   
    # def para publicar a mensagem de aviso            
    def publish_aviso(self, mensagem):
        msg = String()
        msg.data = mensagem
        self.aviso_publisher.publish(msg)
    
    def destroy_node(self):
        self.shutdown_can() 
        super().destroy_node()

    def shutdown_can(self):
        if self.can_bus:
            self.can_bus.shutdown()
            self.can_bus = None

def main(args=None):
    rclpy.init(args=args)

    distance_min_finder = None
    try:
        distance_min_finder = DistanceMinFinder()
        rclpy.spin(distance_min_finder)

    except KeyboardInterrupt:
        print("Interrompido pelo usuário")
    finally:
        if distance_min_finder:
            distance_min_finder.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"Erro ao desligar: {e}")

if __name__ == '__main__':
    main()
        