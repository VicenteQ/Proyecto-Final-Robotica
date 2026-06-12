from controller import Robot
import math
import heapq
import csv

# 1. Inicialización de Dispositivos
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# --- PREPARACIÓN DEL ARCHIVO CSV ---
archivo_csv = open('datos_trayectoria_escenario2.csv', mode='w', newline='')
writer = csv.writer(archivo_csv)
writer.writerow(['Tiempo_s', 'X_Robot', 'Y_Robot', 'Angulo_Grados', 'Estado_Robot'])
tiempo_simulacion = 0.0

print("Comenzando navegación y grabación de datos en Escenario 2...")

motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')
motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

sensor_frontal_izq = robot.getDevice('ps7')  
sensor_frontal_der = robot.getDevice('ps0')
sensor_lateral_izq = robot.getDevice('ps5')
sensor_lateral_der = robot.getDevice('ps2')

for sensor in [sensor_frontal_izq, sensor_frontal_der, sensor_lateral_izq, sensor_lateral_der]:
    sensor.enable(TIME_STEP)

encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(TIME_STEP)
encoder_derecho.enable(TIME_STEP)

# --- 2. CONFIGURACIÓN DEL MAPA Y RUTA (A*) ---
# Inicio en [0,0] y Meta en [3,3]
mapa_grilla = [
    [0, 1, 0, 0],  # Fila 0: El primer elemento es el Inicio [0,0]
    [0, 1, 0, 1], 
    [0, 0, 0, 1], 
    [1, 1, 0, 0]   # Fila 3: El último elemento es la Meta [3,3]
]

nodo_inicio = (0, 0) # Esquina superior izquierda
nodo_meta = (3, 3)   # Esquina inferior derecha

def a_star(inicio, meta, grilla):
    filas = len(grilla)
    cols = len(grilla[0])
    frontera = []
    heapq.heappush(frontera, (0, inicio))
    costo_acumulado = {inicio: 0}
    camino = {inicio: None}
    
    while frontera:
        _, actual = heapq.heappop(frontera)
        if actual == meta:
            break
        direcciones = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for df, dc in direcciones:
            nf, nc = actual[0] + df, actual[1] + dc
            if 0 <= nf < filas and 0 <= nc < cols and grilla[nf][nc] == 0:
                nuevo_costo = costo_acumulado[actual] + 1
                if (nf, nc) not in costo_acumulado or nuevo_costo < costo_acumulado[(nf, nc)]:
                    costo_acumulado[(nf, nc)] = nuevo_costo
                    prioridad = nuevo_costo + (abs(meta[0] - nf) + abs(meta[1] - nc))
                    heapq.heappush(frontera, (prioridad, (nf, nc)))
                    camino[(nf, nc)] = actual
    ruta = []
    actual = meta
    if meta not in camino:
        print("¡ERROR: No hay ruta posible hacia la meta!")
        return []
    while actual != inicio:
        ruta.append(actual)
        actual = camino[actual]
    ruta.reverse()
    return ruta

ruta_celdas = a_star(nodo_inicio, nodo_meta, mapa_grilla)
print(f"Ruta calculada en celdas: {ruta_celdas}")

ruta_waypoints = []
for (fila, col) in ruta_celdas:
    # Ajuste de conversión: Fila 0 es Y positivo (arriba), Col 0 es X negativo (izquierda)
    x_real = (col * 0.25) - 0.375
    y_real = 0.375 - (fila * 0.25) 
    ruta_waypoints.append((x_real, y_real))

print(f"Coordenadas reales de la ruta: {ruta_waypoints}")

# --- 3. VARIABLES GLOBALES (Cinemática y Odometría) ---
RADIO_RUEDA = 0.0205 
DISTANCIA_RUEDAS = 0.053
pos_izq_ant = 0.0
pos_der_ant = 0.0

# INICIALIZACIÓN SIN DESFASE 
# El robot ahora nacerá mentalmente en la esquina superior izquierda
x_robot = (nodo_inicio[1] * 0.25) - 0.375
y_robot = 0.375 - (nodo_inicio[0] * 0.25)

# ATENCIÓN: Al iniciar en la esquina superior izquierda [0,0], 
# el robot arranca mirando hacia el Sur (abajo en la matriz).
phi_robot = -math.pi / 2 

VENTANA = 5
buffer_mediciones = []
d_estimada = 0.08  
P = 1.0            
R = 0.05           
Q = 0.001          
VELOCIDAD_MAX = 3.14
contador_giro = 0

indice_waypoint_actual = 0
TOLERANCIA_LLEGADA = 0.08 # Margen de 8cm

# --- 4. CICLO PRINCIPAL ---
while robot.step(TIME_STEP) != -1:
    
    # --- A. ODOMETRÍA ---
    pos_izq_act = encoder_izquierdo.getValue()
    pos_der_act = encoder_derecho.getValue()
    delta_theta_izq = pos_izq_act - pos_izq_ant
    delta_theta_der = pos_der_act - pos_der_ant
    delta_s_izq = RADIO_RUEDA * delta_theta_izq
    delta_s_der = RADIO_RUEDA * delta_theta_der
    
    delta_s = (delta_s_der + delta_s_izq) / 2.0
    delta_phi = (delta_s_der - delta_s_izq) / DISTANCIA_RUEDAS
    
    x_robot = x_robot + delta_s * math.cos(phi_robot + (delta_phi / 2.0))
    y_robot = y_robot + delta_s * math.sin(phi_robot + (delta_phi / 2.0))
    phi_robot = phi_robot + delta_phi
    
    phi_robot = math.atan2(math.sin(phi_robot), math.cos(phi_robot))
    
    pos_izq_ant = pos_izq_act
    pos_der_ant = pos_der_act

    # --- B. KALMAN ---
    lectura_cruda_izq = sensor_frontal_izq.getValue()
    lectura_cruda_der = sensor_frontal_der.getValue()
    promedio_crudo = (lectura_cruda_izq + lectura_cruda_der) / 2.0
    crudo_sin_ruido = max(0.0, promedio_crudo - 65.0)
    z_k = 0.08 - (crudo_sin_ruido / 300.0) * 0.08
    z_k = max(0.0, min(0.08, z_k)) 
    
    buffer_mediciones.append(z_k)
    if len(buffer_mediciones) > VENTANA: buffer_mediciones.pop(0)
    
    d_predicha_menos = d_estimada - delta_s 
    P_menos = P + Q
    K_k = P_menos / (P_menos + R)
    d_estimada = d_predicha_menos + K_k * (z_k - d_predicha_menos)
    P = (1 - K_k) * P_menos

    # --- C. CONTROL DE COMPORTAMIENTO ---
    UMBRAL_SEGURIDAD = 0.04  # Umbral más bajo para no asustarse con paredes lejanas
    
    # Pre-cálculo de errores para saber si estamos en medio de un giro planificado
    error_angulo = 0.0
    distancia_a_meta = 0.0
    if indice_waypoint_actual < len(ruta_waypoints):
        x_meta, y_meta = ruta_waypoints[indice_waypoint_actual]
        error_x = x_meta - x_robot
        error_y = y_meta - y_robot
        distancia_a_meta = math.sqrt(error_x**2 + error_y**2)
        angulo_deseado = math.atan2(error_y, error_x)
        error_angulo = angulo_deseado - phi_robot
        error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo))
    
    # Regla: Si el A* pide un giro brusco (abs(error_angulo) > 0.3), no evadas
    if d_estimada < UMBRAL_SEGURIDAD and contador_giro == 0 and abs(error_angulo) < 0.3:
        contador_giro = 30  
    
    if contador_giro > 0:
        lat_izq = sensor_lateral_izq.getValue()
        lat_der = sensor_lateral_der.getValue()
        if lat_izq > lat_der + 50:
            motor_izquierdo.setVelocity(VELOCIDAD_MAX)
            motor_derecho.setVelocity(-VELOCIDAD_MAX)
        else:
            motor_izquierdo.setVelocity(-VELOCIDAD_MAX)
            motor_derecho.setVelocity(VELOCIDAD_MAX)
        contador_giro -= 1
        
    else:
        if indice_waypoint_actual < len(ruta_waypoints):
            if distancia_a_meta < TOLERANCIA_LLEGADA:
                print(f"¡Waypoint {indice_waypoint_actual} alcanzado!")
                indice_waypoint_actual += 1
            else:
                # --- CONTROL DE PIVOTE Y DISPARO ---
                if abs(error_angulo) > 0.15: 
                    signo = 1 if error_angulo > 0 else -1
                    v_giro = signo * VELOCIDAD_MAX * 0.5 
                    motor_izquierdo.setVelocity(-v_giro)
                    motor_derecho.setVelocity(v_giro)
                else:
                    motor_izquierdo.setVelocity(VELOCIDAD_MAX * 0.8)
                    motor_derecho.setVelocity(VELOCIDAD_MAX * 0.8)
        else:
            motor_izquierdo.setVelocity(0.0)
            motor_derecho.setVelocity(0.0)
            # Imprimir solo una vez al terminar
            if robot.getTime() % 1.0 < (TIME_STEP/1000.0) and "Completada" not in estado_str:
                 print("¡Ruta Completada! El robot llegó con éxito a la meta.")
            
    # --- D. MONITOREO Y EXPORTACIÓN DE DATOS ---
    tiempo_simulacion += (TIME_STEP / 1000.0)
    phi_grados = math.degrees(phi_robot) % 360
    estado_str = "EVADIENDO" if contador_giro > 0 else f"WP:{indice_waypoint_actual}"
    
    # 1. Escritura continua en el archivo CSV
    writer.writerow([tiempo_simulacion, x_robot, y_robot, phi_grados, estado_str])
    
    # 2. Impresión en consola por segundo
    if robot.getTime() % 1.0 < (TIME_STEP/1000.0): 
        print(f"Pose -> X:{x_robot:.2f} Y:{y_robot:.2f} Ang:{phi_grados:.0f}° | {estado_str}")