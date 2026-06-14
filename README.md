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

## 9. Evaluación Experimental y Análisis de Métricas

A partir de los registros de odometría almacenados en los archivos CSV, se evaluó el desempeño del sistema de navegación en los dos escenarios requeridos. Las métricas extraídas revelan lo siguiente:

* **Análisis Temporal y Eficiencia de Ruta:** Al filtrar el tiempo de inactividad posterior a la llegada a la meta, los datos muestran que el desempeño temporal en ambos escenarios es sumamente eficiente y parejo. En el Escenario 1 (Principal), el robot alcanzó el punto final (WP:6) en 28.54 segundos, recorriendo una distancia física aproximada de 1.28 metros. Por otro lado, en el Escenario 2 (Obstáculos Extra), la adición de obstáculos forzó al algoritmo $A^*$ a trazar una ruta diagonal, logrando completar el recorrido en tan solo 27.13 segundos (una distancia de 1.23 metros). Esto demuestra que una mayor densidad de obstáculos no penaliza el tiempo de viaje si el planificador global encuentra una optimización espacial más directa.
* **Diferencia entre Ruta Planificada y Trayectoria Real:** Al analizar los datos de posición ($X_{Robot}$, $Y_{Robot}$), se evidencia que la curva real del e-puck no es una línea recta perfecta, sino que suaviza los vértices ortogonales de la grilla teórica. Esto se debe a tres factores físicos y lógicos: la inercia física del sistema dinámico, el tiempo de reacción del controlador proporcional (que ajusta las velocidades de las ruedas de forma gradual) y el "inflado" de seguridad asignado a los obstáculos, el cual obliga al algoritmo a planificar márgenes de giro más amplios.
* **Comportamiento de Orientación y Estabilidad de Giro:** La métrica de orientación (`Angulo_Grados`) demuestra un avance en línea recta sumamente estable, con fluctuaciones mínimas durante los trayectos largos. Las caídas abruptas registradas en las gráficas temporales corresponden exclusivamente al fenómeno matemático de *angle wrapping* (cuando el sensor odométrico normaliza el cálculo al cruzar el umbral de los 360° a 0°) y no a oscilaciones mecánicas ni inestabilidad del controlador.
* **Transiciones de Estado y Robustez:** El sistema gestionó la navegación de forma modular, dividiendo la ruta global en sub-metas. La columna de estados lógicos confirmó que el controlador transitó de manera impecable desde WP:0 hasta WP:6 en ambos entornos. El robot no experimentó bloqueos locales, no registró colisiones con los obstáculos añadidos, y detuvo sus motores exitosamente al cumplir la condición de proximidad de la meta final.

## 10. Evidencias Finales: Gráficos y Videos Demostrativos

A continuación, se presentan las gráficas obtenidas mediante el script de análisis en Python y los registros en video de las simulaciones completas para cada entorno de prueba:

### Escenario 1: Entorno Principal (Simple)
* **Gráfico de Trayectoria Espacial (X vs Y):** <img width="819" height="701" alt="image" src="https://github.com/user-attachments/assets/36a7d22a-f44c-46f1-afd6-5ade459f6b04" />

* **Gráfico de Evolución de la Orientación:** <img width="852" height="394" alt="image" src="https://github.com/user-attachments/assets/330cd3f0-e90b-4c3b-9d11-8bd05de9d019" />

* **Video Demostrativo de Ejecución:** 

https://github.com/user-attachments/assets/33921ebd-f368-478f-a7db-1c1b41310568

### Escenario 2: Entorno con Obstáculos Extra (Complejo)
* **Gráfico de Trayectoria Espacial (X vs Y):** <img width="819" height="702" alt="image" src="https://github.com/user-attachments/assets/422ed518-0b8f-4d93-9485-27b18c8679d2" />

* **Gráfico de Evolución de la Orientación:** <img width="852" height="394" alt="image" src="https://github.com/user-attachments/assets/ec892113-8701-4b96-aaad-41ed1bd5401b" />

* **Video Demostrativo de Ejecución:** 

https://github.com/user-attachments/assets/dd2e4049-b63b-4844-a50c-287672c6ccfe

## 11. Instrucciones de Reproducción y Condición Inicial

Para ejecutar y validar la simulación de manera correcta, se deben seguir los siguientes pasos:

1. Abrir el simulador Webots y cargar el mundo correspondiente (`Escenario Inicial.wbt` o `Escenario2.wbt`).
2. **Condición Inicial Crítica:** Para que el cálculo de la odometría funcione de forma perfecta y no se desalinee con la grilla del algoritmo, es obligatorio rotar físicamente al robot e-puck en el entorno 3D para que **quede mirando hacia el "Sur" del mapa (hacia abajo)** antes de presionar el botón de inicio.
3. Presionar *Play* en el simulador. La consola desplegará automáticamente la ruta planificada en celdas, su conversión a coordenadas reales y el paso detallado por cada uno de los *waypoints*.

## 12. Conclusiones y Limitaciones

El desarrollo de este proyecto permitió comprobar que la combinación de un algoritmo de planificación global como $A^*$ junto con un control cinemático por odometría y evasión local reactiva ofrece una solución robusta para la navegación autónoma en entornos estructurados.

* **Limitación del Sistema:** La principal debilidad detectada radica en la dependencia absoluta de los encoders de las ruedas para la estimación de la postura del robot. Al ser un método de odometría pura, el error acumulativo provocado por el deslizamiento de las ruedas con el suelo (*drift*) causaría que en trayectorias de trayectos mucho más largos o prolongados el e-puck se desvíe por completo de los objetivos.
* **Mejoras Futuras:** Para solucionar la deriva odométrica, se propone como mejora implementar un Filtro de Kalman que fusione las lecturas de los encoders con referencias externas, o bien escalar la solución hacia un sistema de SLAM simplificado que use activamente los sensores de distancia para corregir la posición en tiempo real según las paredes detectadas.
