import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time
import can
import struct

#can_bus = can.interface.Bus(channel='can0', interface='socketcan')

#localiza o objeto no plano carteziano 
def localizacaoObjetoCamera(data):
    
    dados = pd.read_csv('/home/fatec/ros2_ws/src/lidar_pkg/scripts/configuracao.cfg')
    
    centro_camera_objeto = dados.iloc[8,1]
            
    linhas = int(dados.iloc[9,1])
    colunas = int(dados.iloc[10,1])
            
    valor_inicial = centro_camera_objeto * 2

    valor_x = valor_inicial / colunas
            
    maximo_x = 1280 / colunas
    maximo_y = 960 / linhas
    
    distancias = []
    
    matriz = []
    for i in range(linhas):
        primeira_coluna = [round(valor_inicial - valor_x * i, 2), round(valor_inicial - valor_x * (i + 1), 2)]
        linha = [primeira_coluna]*colunas
        matriz.append(linha)
    
    if data:
        for obj in data:
            centro_x, centro_y, angulo, tipo = obj
            
            eixo_x = math.floor(centro_x / maximo_x)
            eixo_y = math.floor(centro_y / maximo_y)
            
            valor_distancia = matriz[eixo_y][eixo_x]
            
            distancias.append((valor_distancia, angulo, tipo))
    
    if distancias:
        #print(distancias)
        return distancias
    else:
        distancias = []
        return distancias
    
#coverte as coordenadas polares dos dados cru do lidar para coordenadas carteziana         
def PontosCartezianos(data):
    dados = data
    
    pontos_cartezianos = []
    
    if dados:
        for pontos in dados:
            distancia, angulo = pontos
            
            X = distancia*np.cos(angulo) 
            Y = distancia*np.sin(angulo)
            pontos_cartezianos.append((X,Y))
        
    if pontos_cartezianos:
        return pontos_cartezianos
    else:
        pontos_cartezianos = []
        return pontos_cartezianos

#converte o primeiro ponto, ultimo ponto e ponto central para coordenada polar
def PontosPolares(data):
    pontos = data 
    
    pontos_polares = []
    
    if data:
        for dados in pontos:
            primeiro_ponto, ultimo_ponto, ponto_central = dados
            
            # Coordenadas cartesianas 
            x_primeiro, y_primeiro = primeiro_ponto
            x_ultimo, y_ultimo = ultimo_ponto
            x_central, y_central = ponto_central
            
            # Convertendo 
            r_primeiro = math.sqrt(x_primeiro**2 + y_primeiro**2)
            theta_primeiro = math.atan2(y_primeiro, x_primeiro)
            
        
            r_ultimo = math.sqrt(x_ultimo**2 + y_ultimo**2)
            theta_ultimo = math.atan2(y_ultimo, x_ultimo)
            
        
            r_central = math.sqrt(x_central**2 + y_central**2)
            theta_central = math.atan2(y_central, x_central)
            
            pontos_polares.append(((r_primeiro, theta_primeiro), (r_ultimo, theta_ultimo), (r_central, theta_central)))
    
    if pontos_polares:
        return pontos_polares
    else:
        pontos_polares = []
        return pontos_polares
    
#agrupa os pontos usando geometria eucludiana e devolve: primeiro ponto, ultimo ponto e o ponto central do obj.
def Agrupamento(data):
    obj = []
    obj_atual = []
    
    dados = pd.read_csv('/home/fatec/ros2_ws/src/lidar_pkg/scripts/configuracao.cfg')
    
    #distancia_eucludiana_parametro = 0.4 
    distancia_eucludiana_parametro = float(dados.iloc[11,1])
    
    if data:
        i = 0
        while i + 1 < len(data):
            distancia_x, distancia_y = data[i]
            proxima_distancia_x, proxima_distancia_y = data[i + 1]
            
            valor_x = (abs(distancia_x - proxima_distancia_x))**2
            valor_y = (abs(distancia_y - proxima_distancia_y))**2
            
            distancia_eucludiana = math.sqrt(valor_x + valor_y)
            
            if distancia_eucludiana <= distancia_eucludiana_parametro:
                obj_atual.append((distancia_x, distancia_y))
            else:
                # Caso contrário, finaliza o objeto atual e inicia um novo objeto
                obj.append(obj_atual)
                obj_atual = []   
            
            i += 1
    
    # Adiciona o último objeto atual à lista de objetos
    if obj_atual:
        obj.append(obj_atual)
        
        # Extrai as informações desejadas para cada objeto
    objetos_info = []
    for objeto in obj:
        if objeto:  
            primeiro_ponto = objeto[0]
            ultimo_ponto = objeto[-1]
            soma_x = sum(p[0] for p in objeto)  # Soma das coordenadas x
            soma_y = sum(p[1] for p in objeto)  # Soma das coordenadas y
            ponto_central = (soma_x / len(objeto), soma_y / len(objeto))
                        
            objetos_info.append((primeiro_ponto, ultimo_ponto, ponto_central))

    if objetos_info:
        return objetos_info
    else:
        objetos_info = []
        return objetos_info

#define a localizacao geografica do objeto
def Localizacao(data):
    pontos = data
    
    dados = pd.read_csv('/home/fatec/ros2_ws/src/lidar_pkg/scripts/configuracao.cfg')
    largura_tunel = float(dados.iloc[3,1])
    
    #variaveis para armazenar o valores maximos dos tuneis
    tunel1_max = float(dados.iloc[4,1])
    tunel2_max = float(dados.iloc[5,1])
    tunel3_max = float(dados.iloc[6,1])
    tunel4_max = float(dados.iloc[7,1])
    distancia_zona = float(dados.iloc[2,1])
    
    objetos = []
    
    obj_atual = []
    
    if pontos:
        for obj in pontos:
            # Extraindo os pontos polares do objeto atual
            primeiro_ponto_pol, ultimo_ponto_pol, ponto_central_pol = obj
            
            # Extraindo os componentes de cada ponto polar
            r_primeiro, theta_primeiro = primeiro_ponto_pol
            r_ultimo, theta_ultimo = ultimo_ponto_pol
            r_central, theta_central = ponto_central_pol
            
            angulo_entre_pontos = abs(theta_primeiro - theta_ultimo)
            angulo_corda = angulo_entre_pontos / 2
            distancia_corda = 2 * r_central
            corda = distancia_corda * math.sin(angulo_corda)
            corda_2 = math.trunc(corda * 1000)
            tamanho = int(corda_2 / 100)
            
            angulo_central_graus = math.degrees(theta_central)
            
            distancia_x = abs(math.sin(theta_central) * r_central)
            
            if r_central > 0:
                if distancia_x <= largura_tunel:
                    if r_central <= distancia_zona:
                        risco = 6
                        label = 'ZE'
                        tipo = 'desconhecido'
                        obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
                    
                    elif distancia_zona < r_central <= tunel1_max:
                        risco = 5
                        label = 'T1'
                        tipo = 'desconhecido'
                        obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
                
                    elif tunel1_max < r_central <= tunel2_max:
                        risco = 4
                        label = 'T2'
                        tipo = 'desconhecido'
                        obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
                    
                    elif tunel2_max < r_central <= tunel3_max:
                        risco = 3
                        label = 'T3'
                        tipo = 'desconhecido'
                        obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))

                    elif tunel3_max < r_central <= tunel4_max:
                        risco = 2
                        label = 'T4'
                        tipo = 'desconhecido'
                        obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
                    
                elif distancia_x > largura_tunel and theta_central > 0:
                    risco = 1
                    label = 'LE'
                    tipo = 'desconhecido'
                    obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
                
                elif distancia_x > largura_tunel and theta_central < 0:
                    risco = 1
                    label = 'LD'
                    tipo = 'desconhecido'
                    obj_atual.append((r_central, angulo_central_graus, tamanho, risco, label, tipo))
            
    if obj_atual:
        return obj_atual
    else:
        obj_atual = []
        return obj_atual
    
#def para juntar as informacoes do lidar e da camera caso o objeto seja o mesmo
def FusaoLidarCamera(lidar, camera):
    
    # lista vazia para armazenar os objetos
    deteccao = []
    
    dados_camera = camera
    
    a = 0
    b = 0
    
    if lidar:
        
        for idx_lidar, obj in enumerate(lidar):
            
            if camera:
                for idx_camera, resultado in enumerate(dados_camera):
                    
                    valor_distancia = resultado[0]
                    distancia_min = valor_distancia[1]
                    distancia_max = valor_distancia[0]
                    
                    if distancia_min <= obj[0] <= distancia_max:
                        a = 1   
                    else:
                        a = 0
                    
                    if (obj[1] * resultado[1]) > 0:
                        b = 1
                    else:
                        b = 0
                    
                    if a == 1 and b == 1:
                        distancia_mm = obj[0] * 1000
                        distancia_int = math.trunc(distancia_mm)
                        deteccao.append((distancia_int, math.trunc(obj[1]), obj[2], obj[3], obj[4], resultado[2]))
                        del(dados_camera[idx_camera])
                        break
    
                else:
                    distancia_mm = obj[0] * 1000
                    distancia_int = math.trunc(distancia_mm)
                    deteccao.append((distancia_int, math.trunc(obj[1]), obj[2], obj[3], obj[4], obj[5]))
                        
            else:
                distancia_mm = obj[0] * 1000
                distancia_int = math.trunc(distancia_mm)
                deteccao.append((distancia_int, math.trunc(obj[1]), obj[2], obj[3], obj[4], obj[5]))
                
        if dados_camera:
            for camera1 in dados_camera:
                corda = 0
                risco = 0
                localizacao = 'desconhecido'
                distancia_int = math.trunc(distancia_min*1000)
                angulo = math.floor(camera1[1])
                deteccao.append((distancia_int, angulo, corda, risco, localizacao, camera1[2]))    
    
    elif camera:
        
        for resultado in camera:
            valor_distancia = resultado[0]
            distancia_min = math.trunc(valor_distancia[1] * 1000)
            distancia_max = valor_distancia[0]
            angulo = math.floor(resultado[1])
                    
            corda = 0
            risco = 0
            localizacao = 'desconhecido'
                
            deteccao.append((distancia_min, angulo, corda, risco, localizacao, resultado[2]))
            
    if deteccao:
        #print(deteccao)
        return deteccao
    else:
        corda = 0
        risco = 0
        localizacao = 'desconhecido'
        distancia = 0
        angulo = 0
        tipo = 'desconhecido'
        deteccao.append(((distancia), (angulo), (corda), risco, localizacao, tipo))
        return deteccao                    

#def para plotar os pontos do agrupamento, somente do lidar
def plotar_pontos(objetos):
    plt.figure(figsize=(8, 6))
    
    for objeto in objetos:
        pontos_para_plotar = []
        for ponto in objeto:
            pontos_para_plotar.append(ponto)
        pontos_para_plotar = np.array(pontos_para_plotar)
        
        plt.plot(pontos_para_plotar[:, 0], pontos_para_plotar[:, 1], marker='o', linestyle='-')
    
    plt.title('Pontos Ligados do Agrupamento')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

#def para enviar as mensagens can
def send_can_message(can_bus, msg_id, data):
    # Cria uma interface CAN com o canal especificado
    #can_bus = can.interface.Bus(channel=channel, interface='socketcan')
    
    try:
        # Cria uma mensagem CAN
        message = can.Message(arbitration_id=msg_id,
                            data=data,
                            is_extended_id=True)
        # Envia a mensagem
        can_bus.send(message)
        time.sleep(0.01)
        #data_hex = ' '.join(f'{byte:02X}' for byte in data)
        #print(f"Mensagem CAN enviada: ID = {msg_id}, Dados = {data_hex}")
    except can.CanError:
        #self.get_logger().error("Falha ao enviar a mensagem CAN")
        reset_can_bus()
    """
    # Cria uma mensagem CAN
    message = can.Message(arbitration_id=msg_id,
                        data=data,
                        is_extended_id=True)
    attempts = 0

    while attempts < 5:
        try:
            can_bus.send(message)
            time.sleep(0.01)
            return  # Sai do loop se o envio for bem-sucedido
        except can.CanOperationError:
            attempts += 1
            print("tentativa: ", attempts)
            #get_logger().warning(f"Buffer cheio: {e}. Tentando novamente...")
            time.sleep(1)  # Espera antes de tentar novamente
            if attempts == 5:
                can_bus = reset_can_bus(can_bus)  # Reinicializa se necessário
                if can_bus is None:
                    print("Falha ao reinicializar o barramento CAN.")
                    return
    """
    
#def que formata os dados para enviar via can
def format_data(idx, localizacao, distancia, angulo, tipo, tamanho, risco):
    # Conversão dos dados para bytes
    data = bytearray()
    #little-endian formato dos dados
    data.extend(struct.pack('<B', idx))
    data.extend(struct.pack('<B', localizacao)) 
    data.extend(struct.pack('<H', distancia)) #<H 2 bytes, <B 1 byte
    data.extend(struct.pack('<b', angulo)) # b minusculo para numeros negativos, -128 ate 127
    data.extend(struct.pack('<B', tipo))
    data.extend(struct.pack('<B', tamanho)) 
    data.extend(struct.pack('<B', risco))

    return data

#def para atribuir um valor a localização do objeto, uma codificação para diminuir seu tamanho
def localizacao_objeto(local):
    match local:
        case "ZE":
            return 10
        case "T1":
            return 11
        case "T2":
            return 12
        case "T3":
            return 13
        case "T4":
            return 14
        case "LE":
            return 15
        case "LD":
            return 16
        case "desconhecido":
            return 17
        case _:
            return 0

#def para atribuir um valor ao tipo do objeto, uma codificação para diminuir seu tamanho
def tipo_objeto(tipo):
    #print("tipo: ", tipo)
    tipo2 = tipo.strip()
    
    match tipo2:
        case "TF":
            return 20
        case "TM":
            return 21
        case "TD":
            return 22
        case _:
            return 0
            
def tipo_camera(dado):

    caminho_arquivo = '/home/fatec/ros2_ws/src/lidar_pkg/scripts/objetos_bb.cfg'

    df = pd.read_csv(caminho_arquivo, delimiter=',', header=None)


    valor_procurado = dado

    indices = df[df[0].astype(str).str.contains(valor_procurado)].index.tolist()

    #print(f'Índices : {indices}')

    for indice in indices:
        valor_segunda_coluna = df.at[indice, 1]  
        #print(f'no índice {indice}: {valor_segunda_coluna}')
        return valor_segunda_coluna 

def reset_can_bus(can_bus):
    try:
        can_bus.shutdown()
        time.sleep(0.15)
        return can.interface.Bus(channel='can0', interface='socketcan')  # Retorna o novo barramento
    except Exception as e:
        print(f'Erro ao reinicializar o barramento CAN: {e}')
        return None  # Retorna None em caso de falha
