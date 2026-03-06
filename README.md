# ros-repo-2

Pinky 로봇의 웹 제어/주차 기능 핵심 코드 저장소입니다.

## 포함된 핵심 파일
- `pinky_web/pinky_web.py`: 메인 웹 애플리케이션
- `pinky_web/pinky_autopark_node.py`: ROS2 주차 노드
- `pinky_web/autopark/`: 자동주차 로직 모듈
- `pinky_web/run_pinky_web.sh`: 통합 실행 스크립트
- `pinky_web/smoke_test.sh`: HTTP 스모크 테스트
- `pinky_web/pinky-mjpeg.service`: systemd 서비스 파일
- `pinky_web/*.md`: 카메라/주차 관련 운영 문서

## 제외한 파일
보안/운영 민감 정보와 산출물은 제외했습니다.
- `.env`, 로그, 캐시(`__pycache__`), 백업 파일
- 런타임 데이터(`face_db.json`, `greet_state.json`)

## 실행 환경 (요약)
- Ubuntu + Python 3.12
- ROS2 Jazzy
- `curl`, `jq` (스모크 테스트용)
- 로컬 경로 기준으로 작성된 스크립트 포함 (`/home/pinky`)

## 빠른 실행
```bash
cd /home/pinky/ros-repo-2/pinky_web
chmod +x run_pinky_web.sh smoke_test.sh
./run_pinky_web.sh
```

## 동작 확인
```bash
cd /home/pinky/ros-repo-2/pinky_web
./smoke_test.sh
```

## 참고
실서버에서 사용할 환경변수(`.env`)와 모델 파일은 운영 환경에서 별도 배치해야 합니다.
