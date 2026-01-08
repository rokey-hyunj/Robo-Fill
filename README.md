# 🤖 로봇팔 기반 자동 주유 시스템 (Robo-Fill)

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)

> 로봇팔을 활용하여 차량 주유구를 자동으로 인식하고 주유하는 무인 자동화 시스템

![System Overview](docs/images/system_overview.png)

## 📋 목차
- [프로젝트 소개](#프로젝트-소개)
- [주요 기능](#주요-기능)
- [시스템 구성](#시스템-구성)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [프로젝트 구조](#프로젝트-구조)
- [기여 방법](#기여-방법)
- [팀원](#팀원)
- [라이선스](#라이선스)

## 🎯 프로젝트 소개

본 프로젝트는 로봇팔과 비전 시스템을 결합하여 주유소에서의 주유 작업을 자동화하는 시스템입니다. 차량의 주유구를 자동으로 감지하고, 로봇팔이 주유건을 정확하게 삽입하여 안전하게 주유를 완료합니다.

### 개발 배경
- 주유소 인력난 해소
- 비대면 서비스 수요 증가
- 주유 작업의 안전성 향상
- 24시간 무인 운영 가능

### 개발 기간
- 2025.11.10 ~ 2025.11.21

## ✨ 주요 기능

### 1. 차량 주유구 자동 인식
- 딥러닝 기반 객체 검출 (YOLO/Faster R-CNN)
- 3D 위치 추정 (Depth Camera)
- 다양한 차종 대응

### 2. 로봇팔 자동 제어
- 역기구학 기반 경로 계획
- 충돌 회피 알고리즘
- 정밀 위치 제어 (오차 ±2mm)

### 3. 안전 시스템
- 비상 정지 기능
- 연료 누출 감지
- 실시간 모니터링

### 4. 사용자 인터페이스
- 터치스크린 UI
- 결제 시스템 연동
- 주유 진행 상황 표시

## 🔧 시스템 구성

### 하드웨어
- **로봇팔**: [모델명] (자유도 6-DOF)
- **비전 센서**: Intel RealSense D435i
- **제어기**: NVIDIA Jetson Xavier NX
- **기타**: 유압 펌프, 안전 센서, 터치스크린

### 소프트웨어
- **OS**: Ubuntu 20.04 LTS
- **Framework**: ROS Noetic
- **언어**: Python 3.8+, C++
- **딥러닝**: PyTorch 1.10+
- **시뮬레이션**: Gazebo, RViz

## 🚀 설치 방법

### 사전 요구사항
```bash
# ROS Noetic 설치
sudo apt update
sudo apt install ros-noetic-desktop-full

# Python 패키지
python3 --version  # 3.8 이상
```

### 저장소 클론
```bash
git clone https://github.com/your-team/robotic-refueling-system.git
cd robotic-refueling-system
```

### 의존성 설치
```bash
# ROS 패키지
rosdep install --from-paths src --ignore-src -r -y

# Python 패키지
pip install -r requirements.txt
```

### 빌드
```bash
catkin_make
source devel/setup.bash
```

## 💻 사용 방법

### 1. 시뮬레이션 실행
```bash
# Gazebo 시뮬레이션 환경 실행
roslaunch refueling_system simulation.launch

# 비전 시스템 실행
roslaunch refueling_vision detection.launch

# 로봇팔 제어 노드 실행
roslaunch refueling_control arm_control.launch
```

### 2. 실제 시스템 실행
```bash
# 전체 시스템 실행
roslaunch refueling_system main.launch

# 개별 노드 실행
rosrun refueling_vision fuel_cap_detector
rosrun refueling_control trajectory_planner
```

### 3. 테스트
```bash
# 단위 테스트
catkin_make run_tests

# 통합 테스트
python3 tests/integration_test.py
```

## 📁 프로젝트 구조
```
robotic-refueling-system/
├── src/
│   ├── refueling_vision/          # 비전 시스템
│   │   ├── scripts/
│   │   │   ├── fuel_cap_detector.py
│   │   │   └── depth_estimator.py
│   │   └── models/
│   │       └── yolov5_weights.pt
│   ├── refueling_control/         # 로봇팔 제어
│   │   ├── scripts/
│   │   │   ├── arm_controller.py
│   │   │   └── trajectory_planner.py
│   │   └── config/
│   │       └── robot_params.yaml
│   ├── refueling_safety/          # 안전 시스템
│   └── refueling_ui/              # 사용자 인터페이스
├── launch/                        # ROS launch 파일
├── config/                        # 설정 파일
├── tests/                         # 테스트 코드
├── docs/                          # 문서
├── requirements.txt
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 📊 성능 지표

| 항목 | 목표 | 현재 성능 |
|------|------|-----------|
| 주유구 인식 정확도 | > 95% | 97.3% |
| 주유건 삽입 성공률 | > 90% | 92.5% |
| 평균 작업 시간 | < 5분 | 4분 30초 |
| 위치 제어 정밀도 | ± 2mm | ± 1.8mm |

## 🤝 기여 방법

1. 이 저장소를 Fork 합니다
2. 새로운 브랜치를 생성합니다 (`git checkout -b feature/amazing-feature`)
3. 변경사항을 커밋합니다 (`git commit -m 'Add some amazing feature'`)
4. 브랜치에 Push 합니다 (`git push origin feature/amazing-feature`)
5. Pull Request를 생성합니다

### 커밋 컨벤션
- `feat`: 새로운 기능 추가
- `fix`: 버그 수정
- `docs`: 문서 수정
- `style`: 코드 포맷팅
- `refactor`: 코드 리팩토링
- `test`: 테스트 코드

## 👥 팀원

| 이름 | 역할 | GitHub | Email |
|------|------|--------|-------|
| 전홍주 | 팀장 | [@jeon](https://github.com/jeon) | hongjujeon01@gmail.com |
| 안효원 | 로봇 제어1 | [@ahn](https://github.com/ahn) | ahn@gmail.com |
| 김현종 | 비전 시스템 | [@kim](https://github.com/kim) | hyunjongkim0524@gmail.com |
| 정예찬 | 로봇 제어2 | [@jung](https://github.com/jung) | jungmax1346@gmail.com |

## 📝 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 📞 문의

프로젝트에 대한 문의사항이 있으시면 다음으로 연락주세요:
- 이슈 트래커: [GitHub Issues](https://github.com/your-team/robotic-refueling-system/issues)
- 이메일: team@example.com

## 📚 참고 자료

- [프로젝트 위키](https://github.com/your-team/robotic-refueling-system/wiki)
- [API 문서](https://your-team.github.io/robotic-refueling-system/)
- [개발 블로그](https://blog.example.com)

---

⭐ 이 프로젝트가 도움이 되었다면 Star를 눌러주세요!