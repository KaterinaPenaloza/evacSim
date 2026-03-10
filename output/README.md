# Output

Esta carpeta contiene los archivos CSV generados por la simulación de evacuación.

Cada archivo corresponde a un escenario específico de distribución de velocidades de los agentes.

---

## Identificación de archivos

Los archivos siguen el formato:

```
evacuation_data_escenarioXXXYYYZZZ.csv
```

donde:

- **XXX** → porcentaje de agentes lentos  
- **YYY** → porcentaje de agentes con velocidad normal  
- **ZZZ** → porcentaje de agentes rápidos  

Ejemplo:

```
evacuation_data_escenario000000100.csv
```

Significa:

- 0% agentes lentos  
- 0% agentes normales  
- 100% agentes rápidos  

Otros ejemplos incluidos en el repositorio:

```
evacuation_data_escenario100000000.csv
evacuation_data_escenario000100000.csv
evacuation_data_escenario033033033.csv
evacuation_data_escenario050000050.csv
```

---

## Estructura de los archivos CSV

Cada archivo contiene la evolución de la evacuación a lo largo del tiempo.

Columnas principales:

```
tick
time_seconds
evacuated_count0
evacuated_count1
evacuated_count2
...
```

Descripción:

- **tick** → paso de simulación
- **time_seconds** → tiempo real equivalente en segundos
- **evacuated_countX** → número acumulado de agentes evacuados en el punto o salida X

Estos datos permiten analizar:

- velocidad de evacuación
- comparación entre escenarios
- curvas de evacuación
- tiempo total de evacuación

---

## Uso de los datos

Los archivos CSV generados en esta carpeta son utilizados posteriormente por los scripts de análisis para:

- calcular métricas estadísticas
- generar curvas de evacuación
- comparar escenarios de simulación
- producir gráficos finales del estudio
