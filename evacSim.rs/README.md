# evacsim.rs

Esta carpeta contiene los archivos de configuración del escenario de simulación que utiliza Repast para ejecutar la simulación. 

Estos archivos definen cómo se ejecuta el modelo, incluyendo:

- parámetros de simulación
- configuración del contexto
- visualización del modelo

---

## Descripción de los componentes

### styles/

Contiene los estilos de visualización del modelo, incluyendo la apariencia de los agentes y otros elementos mostrados en el display de la simulación (colores, formas, etc.).


### context.xml

Define el contexto principal del modelo en Repast y las proyecciones espaciales utilizadas por la simulación.

Incluye la proyección geográfica donde se ejecuta el modelo y capas adicionales utilizadas durante la simulación.


### parameters.xml

Define los parámetros configurables de la simulación, tales como:

- número de agentes
- tiempo de pre-evacuación
- distribución de velocidades de los agentes
- semilla aleatoria

Estos parámetros pueden modificarse antes de ejecutar la simulación para generar distintos escenarios experimentales.


### scenario.xml

Archivo principal del escenario.  

Este archivo indica a Repast qué componentes deben cargarse al iniciar la simulación:

- cargador del modelo (`ContextCreator`)
- displays de visualización
- acciones de generación de gráficos
- datasets utilizados durante la simulación

### displays

Los archivos `display` definen cómo se visualiza la simulación en la interfaz de Repast.

- `display_1` muestra una representación simple del modelo.
- `display_2` corresponde al display original del proyecto.
- `display_3` es el display principal utilizado en la simulación, donde se visualiza el mapa y el movimiento de los agentes.

### data loader

Define la clase encargada de inicializar el modelo y crear el contexto de simulación.

### user_path.xml

Define la ruta de la clase del modelo, indicando a Repast dónde encontrar:

- las clases compiladas del modelo
- las bibliotecas necesarias para la ejecución

---

## Rol dentro del proyecto

Esta carpeta es configuración del escenario de simulación, conectando:

- el modelo implementado en Java
- los datos geoespaciales del entorno
- los parámetros experimentales
- las herramientas de visualización y recolección de datos

Repast utiliza estos archivos para cargar y ejecutar el modelo de simulación completo.
