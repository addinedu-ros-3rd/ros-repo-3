# ros-repo-3
파이널 프로젝트 3조 저장소. 사무실 도우미 로봇

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

## 기술스택
<img src="https://img.shields.io/badge/Python3-#3776AB?style=for-the-badge&logo=Python&logoColor=black">


## Use Case Diagram
![Screenshot from 2024-01-02 14-10-56](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/300d21e0-f566-41df-aac1-43a6956aaf1e)

## 로봇팔 상태전이도
![Screenshot from 2024-01-02 14-30-17](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/1669fa5f-473a-4a62-b043-8e910a9c7cd3)

## 로봇팔 기본 구성 요소
![Screenshot from 2024-01-02 14-31-20](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/7b9c7e8b-d9f4-40cd-9649-b8c86d72ec11)

## 비품 가져다 주기 시나리오
![bipoom drawio](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/a4704649-b1ed-47cb-92a2-4745ab69350f)





