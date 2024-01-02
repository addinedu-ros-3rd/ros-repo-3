# ros-repo-3
파이널 프로젝트 3조 저장소. 축사 청소 로봇

## 시스템 구성도
![Screenshot from 2024-01-02 13-53-47](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/491d80cc-f796-467c-8124-3bb47f20dafe)

|Components|Responsibility|
|---|---|
|**SLAM**|map 생성 및 로봇의 위치 추정|
|**Planning**|map 기반 경로 계획 및 청소 완료 판단|
|**Navigation**|청소 시작 및 로봇의 주행에 관련된 동작을 요청|
|**Object Detection**|장애물 탐지 및 청소 상태 판단|
|**DB**|map과 로봇의 이동 trajectory 저장|
|**ROBOT ARM**|모바일로 원격 수동 조작|
|**YD LIDAR**|지도 생성 및 로봇의 위치 추정|
|**ROBOT Camera**|사용자 GUI에 디스플레이 및 Obeject Detection에 image 전달|

## Use Case Diagram
![Screenshot from 2024-01-02 14-10-56](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/300d21e0-f566-41df-aac1-43a6956aaf1e)




