# Análisis

Esta carpeta contiene los scripts utilizados para el **análisis de resultados de las simulaciones de evacuación**.

Los scripts procesan los archivos CSV generados por el simulador basado en agentes y generan métricas estadísticas y visualizaciones que permiten evaluar el comportamiento de distintos escenarios de evacuación.

---

## Funcionalidad

El script principal realiza las siguientes tareas:

- Lectura de archivos CSV generados por el simulador
- Limpieza y procesamiento de datos
- Cálculo de métricas estadísticas:
  - promedio de agentes evacuados
  - desviación estándar entre experimentos
- Generación de visualizaciones:
  - curvas de progreso de evacuación
  - gráficos comparativos entre escenarios
  - boxplots de tiempos de evacuación

Los resultados se exportan como **imágenes PNG de alta resolución**.

---

## Entrada

Archivos CSV generados por el simulador, que contienen:

- `time_seconds`: tiempo de simulación
- columnas `evacuated_count_*`: número de agentes evacuados por experimento

Cada archivo puede contener múltiples ejecuciones del mismo escenario.

---

## Salida

El script genera:

- archivos CSV procesados con estadísticas agregadas
- gráficos de progreso de evacuación
- gráficos comparativos entre escenarios
- boxplots de distribución de tiempos de evacuación

Todos los gráficos se guardan automáticamente en graficos/

---

## Dependencias

El análisis utiliza las siguientes bibliotecas de Python:

- pandas
- numpy
- matplotlib

---

## Proceso

| Flujo de procesamiento |
|---|
| Simulación en Repast |
|     ↓ |
| Resultados CSV (`evacuation_data_escenarioX.csv`) |
|     ↓ |
| Procesamiento de datos (Python) |
|     ↓ |
| CSV procesado (`evacuation_data_escenarioX_procesado.csv`) |
|     ↓ |
| Visualización |
|     ↓ |
| Figuras (`graficos/*.png`) |
---

## Ejecución

```bash
python evacuation_data_analisis.py evacuation_data_escenario000000100.csv evacuation_data_escenario000100000.csv evacuation_data_escenario100000000.csv
