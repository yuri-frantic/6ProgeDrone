# Работа с docker

В этом разделе мы разберём инструкцию для работы с docker-образом. **Вы можете пропустить его, если не планируете использовать контейнер.**


В отличие от образа, который вы рассматривали в прошлом модуле, docker содержит библиотеку OpenCV для Python и C++.

---
**Примечание**

В инструкции к модулю 5 вы найдёте инструкцию по установке docker на свой ПК. В ней подробно описано применение docker.

Если хотите использовать наработки модуля 5 в контейнере, необходимо добавить его при сборке при помощи
`COPY` или `RUN git clone ...` Также можно добавить ещё один `--volume=...` при запуске контейнера.

---

# Установка

```bash
curl -fsSL https://get.docker.com | sh
```

```bash
sudo usermod -aG docker $USER
```

```bash
sudo systemctl disable --now docker.service docker.socket
```

```bash
sudo apt-get install -y uidmap
```

```bash
dockerd-rootless-setuptool.sh install --force
```

# Сборка

Для сборки контейнера необходимо выполнить команду:

```bash
docker build -t px4_simulation:latest .
```

# Запуск

Перед запуском выполните команду `xhost+` для разрешения контейнеру доступу к экрану через **xserver**.

Для запуска предлагается воспользоваться скриптом из папки scripts

```bash
./start_sim_docker.sh -c <container_name>
```

Например:

```bash
./start_sim_docker.sh -c px4_simulation:latest
```