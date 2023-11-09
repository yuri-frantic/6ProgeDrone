## Установка OpenCV
<a name="install_opencv"></a>

![img.png](../.img/img.png)
<p align="center">
    <strong>Источник: </strong> [opencv](https://www.google.com/url?q=https://t3.gstatic.com/images?q%3Dtbn:ANd9GcQkljOsNRcSAiSJ2akd_u_4ZdT-T-oJfXve4EpMTccl5ITwL_pe&sa=D&source=docs&ust=1697192853595280&usg=AOvVaw0OvkKpD5ouobmm6g2op8GP)
</p>

**OpenCV** — это библиотека алгоритмов компьютерного зрения с открытым исходным кодом. Для большинства задач, в том числе навигации БЛА, функционала OpenCV достаточно. Для начала работы необходимо установить библиотеку, выполнив следующие пункты.

### Установка на python

Введите команды:

```bash
pip install opencv-python
```

```bash
pip install opencv-contrib-python
```

### Установка для c++



```bash
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

```bash
sudo apt-get install libjpeg-dev
libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev
libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common
libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev
libopenexr-dev libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev
libavresample-dev
```

Скачайте репозиторий **OpenCV**. Можете сделать это при помощи команды:

```bash
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.8
```

```bash
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 4.8
```

Перейдите в папку OpenCV и создайте директорию для сборки:

```bash
cd ../opencv
mkdir build
```

Создайте файл управления сборкой при помощи cmake:

```bash
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
```

Запустите сборку библиотеки:

```bash
make -j$(nproc)
```
**Сборка может занять некоторое время (в среднем не более 15—20 минут).**


После завершения сборки выполните установку:

```bash
sudo make install
```

Обновляем пути к библиотекам:
```bash
sudo ldconfig
```

### Установка пакетов для работы с OpenCV с использованием ROS

Для работы с изображением через темы (ros topics) необходимо установить
следующие пакеты при помощи команд:

```bash
sudo apt install ros-noetic-cv-bridge
```

```bash
sudo apt install ros-noetic-image-transport
```