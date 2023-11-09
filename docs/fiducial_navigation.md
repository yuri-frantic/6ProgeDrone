# Создание навигационной системы БЛА с использованием реперных маркеров

![img.png](../.img/aruco_orign.png)
<p align="center">
    <strong>Источник: </strong> Михаил Колодочка
</p>

Если систему позиционирования ЛА создают при помощи оптического потока, рассматривают следующие СК: связная СК (body frame), СК камеры (camera frame), СК реперного маркера или массива реперных маркеров (fiducial frame).


---

Примеры создания и использования поля реперных маркеров были рассмотрены нами в [материале](visual_navigation_basics.md). Поэтому при необходимости рекомендуем ознакомиться с ними.

---


### Передача данных в автопилот

Когда сторонняя навигационная система, например с реперными маркерами на борту, получает данные о положении, то отправляет их в топик **/mavros/vision_pose/pose** в формате сообщений
**geometry_msgs/PoseStamped** или **geometry_msgs/PoseWithCovarianceStamped**.
В таком сообщении, помимо вектора положения и кватерниона ориентации, указана ковариационная матрица размерности 6 × 6 положения и углов ориентации. Мы задали дисперсию для реперных маркеров через константу в конфигураторе. Также можно добавить расчёт СКО навигационной системы, если это позволяют размеры области изображения, занимаемого реперным маркером. 

Чем меньше пикселей приходится на реперный маркер, тем больше СКО определения положения и ориентации.

Вы уже знакомы со структурой таких сообщений по модулю 5. Передавая сообщения, присваивайте метку времени к заголовку сообщения header.stamp. Без неё автопилот не сможет использовать визуальную навигацию.

---
### Примечание

Сначала реперные маркеры не будут в поле зрения, так как камера расположена близко к земле.


Так как фильтру Калмана необходим постоянный поток данных, важно передавать последнее увиденное значение или нули для X и Y (0,0), если маркер не появился в поле зрения.я.

---


### Предлагаемая архитектура программы

```mermaid
    flowchart TD
        mavros(mavros node) <--mavlink--> px4(PX4 Autopilot SITL)
        visual_navigation_node -- PoseStamped ros--> mavros
        gazebo_sim(Симулятор Gazebo) --Image ros--> visual_navigation_node
```
<p align="center">
    <strong>Источник: </strong> Михаил Колодочка
</p>

```mermaid
classDiagram
  direction RL
  class BoardParams {
      + float markerSeparation;
      + float markerSize;
      + int markerX;
      + int markerY;
  }
  BoardParams -- FiducialNavigationNode
  class FiducialNavigationNode {
    - imageSub_ : ros::Subscriber
    - visionPosePub_ : ros::Publisher
    - node_ : ros::NodeHandle&
    - visionPoseMsg_ : geometry_msgs::PoseStamped
    - init() : void
    - params_ : BoardParams
    - calculatePosition(cv::Mat) : bool
    - imageSubscriber(sensor_msgs::Image) : void
    + FiducialNavigationNode(ros::NodeHandle&, BoardParams) : FiducialNavigationNode
  }
```
<p align="center">
    <strong>Источник: </strong> Михаил Колодочка
</p>

### Вычислительный алгоритм

1) Получение изображения.

2) Поиск реперных маркеров.

3) Определение положения и ориентации.

4) Перевод из СК камеры в СК маркера и от камеры в связную СК.

5) Отправка данных на автопилот.
