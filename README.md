# Proyecto Final: Navegación Autónoma con Planificación de Rutas

---

## 1. Integrantes del Grupo
- Vicente Aburto Falcón
- Yamil Soleman Fernandez
- Vicente Nills Quezada Gallardo
- Sebastián García Valdebenito
- Ignacio Matus de la Parra

## 2. Línea Seleccionada
**Línea A: Planificación de rutas.**
El desarrollo se centra en la navegación desde una posición inicial hasta una meta dentro de un entorno con obstáculos, utilizando el algoritmo de planificación $A^*$ sobre una grilla de ocupación discreta.

## 3. Objetivo del Proyecto
Diseñar, implementar y evaluar en el simulador Webots un sistema de navegación autónoma para un robot móvil diferencial. Este sistema integra control cinemático, percepción sensorial, estimación de movimiento mediante odometría y toma de decisiones utilizando planificación de rutas global complementada con evasión reactiva de obstáculos local.

## 4. Descripción del Robot, Sensores y Actuadores
El proyecto utiliza el robot móvil diferencial **e-puck** estándar de Webots.
* **Actuadores:** Dos motores de tracción independientes (`left wheel motor` y `right wheel motor`) que permiten el control de movimiento diferencial (avanzar, girar y seguir trayectorias).
* **Sensores Infrarrojos (Percepción):** Se emplean los sensores frontales (`ps7` y `ps0`) para medir la distancia hacia obstáculos inmediatos, y los sensores laterales (`ps5` y `ps2`) para asistir en la decisión de la dirección de giro durante la evasión reactiva.
* **Encoders (Estimación):** Sensores de posición angular en ambas ruedas (`left wheel sensor` y `right wheel sensor`) utilizados para estimar el desplazamiento y la orientación del robot mediante odometría.

## 5. Descripción de los Escenarios de Prueba
El sistema ha sido evaluado en dos entornos diseñados en Webots:
* **Escenario 1 (Simple):** Correspondiente al archivo `Escenario Inicial.wbt`. Es un entorno de baja complejidad, compuesto por una matriz de 4x4 (celdas de 0.25m) con una distribución mínima de obstáculos (cajas de madera), permitiendo validar la ruta base generada por el algoritmo.
* **Escenario 2 (Complejo):** Correspondiente al archivo `Escenario2.wbt`. Presenta una mayor densidad de obstáculos y zonas bloqueadas, lo que exige mayor precisión en el seguimiento de los *waypoints* generados por la grilla y pone a prueba el sistema de evasión reactivo para evitar colisiones.

## 6. Explicación del Algoritmo Implementado
La solución es un sistema híbrido que combina navegación global y local:
1.  **Navegación Global ($A^*$):** El entorno se mapea en una matriz bidimensional (grilla de ocupación) donde los 0 son espacios libres y los 1 son obstáculos. El algoritmo $A^*$ calcula la ruta más corta desde el nodo inicial (0,0) hasta la meta (3,3) evaluando el costo de movimiento y la heurística de distancia Manhattan. Esta ruta se traduce en coordenadas espaciales ($X, Y$) que actúan como *waypoints*.
2.  **Odometría:** Las lecturas de los encoders se utilizan para actualizar de manera iterativa la postura del robot ($X, Y, \phi$) calculando la diferencia del desplazamiento lineal ($\Delta s$) y angular ($\Delta \phi$).
3.  **Filtrado y Estimación:** Se utiliza un filtro de Kalman unidimensional que fusiona la predicción del modelo cinemático con las lecturas de los sensores infrarrojos frontales para estimar la distancia real a los obstáculos, ignorando falsos positivos.
4.  **Control Reactivo:** Si la distancia estimada al obstáculo cae por debajo de un umbral de seguridad (0.04m), el robot interrumpe su trayectoria hacia el *waypoint*, lee los sensores laterales y ejecuta un giro ininterrumpido hacia la dirección más despejada durante un número fijo de iteraciones antes de retomar su curso original.

## 7. Pseudocódigo de la Solución
```text
INICIALIZAR motores, sensores, encoders
DEFINIR mapa_grilla, nodo_inicio, nodo_meta

ruta_waypoints = A_STAR(mapa_grilla, nodo_inicio, nodo_meta)
ESTADO = NAVEGANDO

MIENTRAS simulacion_activa:
    LEER encoders
    ACTUALIZAR odometria (x, y, phi)
    
    LEER sensores_frontales
    estimacion_distancia = FILTRO_KALMAN(lecturas_frontales, avance_odometria)
    
    SI estimacion_distancia < UMBRAL y ESTADO != EVADIENDO:
        ESTADO = EVADIENDO
        temporizador_giro = 30
        DECIDIR direccion_giro basandose en sensores_laterales
        
    SI ESTADO == EVADIENDO:
        GIRAR en direccion_giro
        temporizador_giro -= 1
        SI temporizador_giro == 0: ESTADO = NAVEGANDO
        
    SINO (ESTADO == NAVEGANDO):
        SI waypoint_actual alcanzado:
            AVANZAR al siguiente waypoint
        SINO:
            CALCULAR error_angulo hacia waypoint
            SI abs(error_angulo) > limite:
                PIVOTAR hacia el waypoint
            SINO:
                AVANZAR hacia adelante
```
## 8. Relación Explícita con los Laboratorios 1 y 2
Este proyecto actúa como una extensión e integración directa de lo realizado en los laboratorios previos:
* **Relación con el Laboratorio 1:** Se reutiliza íntegramente el modelo cinemático diferencial implementado previamente. El cálculo del avance ($\Delta s_r, \Delta s_l, \Delta s$) y la actualización de la postura de odometría ($X, Y, \phi$) son la base matemática para que el robot sepa dónde se encuentra en relación con la matriz generada por el algoritmo $A^*$. Asimismo, la lógica de pivotar y avanzar (ajustando $v_l$ y $v_r$) es una evolución del ejercicio de "dibujar un cuadrado".
* **Relación con el Laboratorio 2:** Se implementa el mismo sistema de percepción robusta, utilizando la ventana móvil y el filtro de Kalman escalar para estabilizar las ruidosas mediciones infrarrojas. Además, se hereda la lógica de navegación reactiva y la máquina de estados con su respectiva "memoria de dirección", lo que permite al e-puck evadir los bloques de madera en los escenarios esquivando obstáculos de manera fluida y sin entrar en bucles de oscilación u chocar.
