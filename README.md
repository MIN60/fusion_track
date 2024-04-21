# fusion_track

fusion_track은 센서퓨전을 이용한 협로주행 코드입니다.

![image](https://github.com/MIN60/fusion_track/assets/49427080/09c630b1-89d2-4747-84b6-c1c9dbcf642b)

노란색, 파란색 라바콘으로 이루어진 길을 따라 주행합니다.

![image](https://github.com/MIN60/fusion_track/assets/49427080/1a237ced-fa6e-4b28-af87-bb9a78208709)

센서퓨전을 이용하여 라바콘을 색상별로 구분합니다.

![image](https://github.com/MIN60/fusion_track/assets/49427080/f4dd15c3-552b-4c1a-9a70-532a0881f664)

두 색상의 라바콘이 모두 감지될 경우 두 라바콘 사이에 중점을 추종하여 주행합니다.

카메라 시야각으로 인해 한 가지 색상의 라바콘만 감지될 경우 반대편 대칭점에 다른 색상의 라바콘이 있다고 가정합니다.

rviz를 통해 시각화된 마커를 확인할 수 있습니다.

# 실행방법

```C++
rosrun fusion_track fusion_track_node
```
