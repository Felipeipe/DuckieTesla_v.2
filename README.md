# DuckieTesla Reloaded

![Duckietown Engineering Chile](https://github.com/Felipeipe/bitacoras-2023/blob/main/img/duckietown_engineering_chile.png?raw=true)

## Intro

Este es un proyecto el cual consiste en la conducción autonoma de un duckiebot. Dicho proyecto utiliza imitation learning y una serie de librerias de python.

## Requisitos

### Hardware

- Duckiebot Modelo DB21J con Sd flasheada. Si no se ha flasheado aun, puedes acceder a [este tutorial](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_sd_card/cli.html), pero habiendo hecho la instalación de DTS, mostrado en los pasos siguientes.

- Pc con Ubuntu 20.04 instalado, con 40 GB de espacio libre extra, para los datos de entrenamiento

### Software e instalación de dependencias

- Python
- ROS
- Tensorflow
- Docker
- DuckieTown Shell
- OpenCV
- Matplotlib

#### Para la instalación de dependencias

En la terminal:

### Python3

```Bash
sudo apt update
sudo apt install python3
```

### Librerias de python3

```Bash
pip install tensorflow
pip install opencv-python
pip install matplotlib
```

### DTS

Para la instalación de Duckietown Shell, debe seguirse una serie de pasos para poder configurarlo correctamente. Debe seguir al pie de la letra [este tutorial](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_laptop/setup_dependencies.html). Debes completar las secciones de [*laptop setup*](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_laptop/index.html) y de [*accounts*](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_account/index.html) para luego seguir el readme de [este repositorio](https://github.com/duckietown/duckietown-lx) hasta la sección 2.

Para poder flashear la sd, debes ejecutar este comando:

```Bash
dts init_sd_card --hostname <Nombre de duckie> --type duckiebot --configuration DB21J --wifi <WIFI>:<WIFI-password>
```

### ROS

Seguir [este tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu). Si no sabes como modificar el archivo .bashrc, debes seguir [este otro tutorial](https://www.digitalocean.com/community/tutorials/bashrc-file-in-linux)
