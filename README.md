# ros-repo-3
개인서비스용 로봇 '에티(ETTI)' Future of Companion Robotics : Human-Robot Interaction with ROS2 and LLM

### 에티(ETTI)

Entertainment : 여가, 오락, 또는 즐거음을 제공하는 활동이나 이벤트를 의미
Talk : 커뮤니케이션과 대화의 요소, 인간과 사물간의 대화
Technology : 현대 기술, 특히 디지털 또는 전자 기술의 사용과 발전
Interaction : 사용자와 시스템 또는 사람들 간의 상호작용

### Project goal
Human-Robot Interaction을 기반으로 사용자와 소통하는 지능형 서비스 로봇 만들기 

### 목표 기능 

- 의사 결정 (👍Yes, 👎No)

- 감정 표현 (😁행복, 😭슬픔, 😡화남)

- 자율 주행 (네비게이션, 장애물 회피, 객체 트래킹 및 따라가기) 

### 에티와(ETTI)와 대화하기
<img src="~/cd/Home/Video/Screencasts/Screencast from 01-25-2024 11:39:11 AM.webm" width="90%"></img>



## 시스템 구성도
![Screenshot from 2024-01-02 13-53-47](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/491d80cc-f796-467c-8124-3bb47f20dafe)

|Components|Responsibility|
|---|---|
|**SLAM**|map 생성 및 로봇의 위치 추정|
|**Planning**|map 기반 경로 계획 및 청소 완료 판단|
|**Navigation**|청소 시작 및 로봇의 주행에 관련된 동작을 요청|
|**Object Detection**|장애물 탐지 및 청소 상태 판단|
|**DB**|map과 로봇의 이동 trajectory 저장|
|**ROBOT ARM (Open Manipulator)**|모바일로 원격 수동 조작|
|**YD LIDAR**|지도 생성 및 로봇의 위치 추정|
|**ROBOT Camera**|사용자 GUI에 디스플레이 및 Obeject Detection에 image 전달|

## 기술스택
#### Laungauge : <img src="https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=Python&logoColor=white"/> <img src="https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white"/> <img src="https://img.shields.io/badge/JavaScript-F7DF1E?style=flat-square&logo=javascript&logoColor=black"/>

#### API : <img src="https://img.shields.io/badge/Google Cloud-4285F4?style=flat-square&logo=Google Cloud&logoColor=white"/> Google Cloud TTS API, Web Speech API

#### DevOps & Tool : <img src="https://img.shields.io/badge/Amazon AWS-232F3E?style=flat-square&logo=amazonaws&logoColor=white"/> <img src="https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=GitHub&logoColor=white"/> <img src="https://img.shields.io/badge/MySQL-4479A1?style=flat-square&logo=MySQL&logoColor=white"/> <img src="https://img.shields.io/badge/Docker-2496ED?style=flat-square&logo=Docker&logoColor=white"/> <img src="https://img.shields.io/badge/Jira-0052CC?style=flat-square&logo=Jira&logoColor=white"/>

#### Connection : <img src="https://img.shields.io/badge/ROS-22314E?style=flat-square&logo=ROS&logoColor=white"/> SSH

#### OS : <img src="https://img.shields.io/badge/Ubuntu-E95420?style=flat-square&logo=Ubuntu&logoColor=white"/>

#### FrameWork : <img src="https://img.shields.io/badge/Flask-000000?style=flat-square&logo=flask&logoColor=white"/> <img src="https://img.shields.io/badge/OpenAI-412991?style=flat-square&logo=OpenAI&logoColor=white"/>

### Robot Hardware 
Open Manipulator, RealSense d435






## Use Case Diagram
![Screenshot from 2024-01-02 14-10-56](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/300d21e0-f566-41df-aac1-43a6956aaf1e)

## 로봇팔 상태전이도
![Screenshot from 2024-01-02 14-30-17](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/1669fa5f-473a-4a62-b043-8e910a9c7cd3)

## 로봇팔 기본 구성 요소
![Screenshot from 2024-01-02 14-31-20](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/7b9c7e8b-d9f4-40cd-9649-b8c86d72ec11)

## 비품 가져다 주기 시나리오
![bipoom drawio](https://github.com/addinedu-ros-3rd/ros-repo-3/assets/146153568/a4704649-b1ed-47cb-92a2-4745ab69350f)





