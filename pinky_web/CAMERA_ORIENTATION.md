# Pinky Camera Orientation Runbook

## 목적
- 카메라 뒤집기 설정 혼선을 방지한다.
- 실행 중 화면 방향 문제를 재부팅 없이 빠르게 해결한다.
- 바닥 기하 캘리브레이션은 `FLOOR_GEOMETRY_CALIBRATION.md`를 사용한다.

## Single Source of Truth
- 카메라 방향은 오직 `/home/pinky/pinky_web/.env`의 `PINKY_CAM_FLIP`으로만 제어한다.
- `run_pinky_web.sh`는 `.env`를 읽어 `--hflip/--vflip` 인자를 결정한다.

## 설정값 의미
- `none`: 뒤집기 없음
- `h`: 좌우 반전
- `v`: 상하 반전
- `hv`: 좌우+상하 반전(180도)

## 변경 절차 (정식)
1. `/home/pinky/pinky_web/.env`에서 `PINKY_CAM_FLIP` 값 변경
2. 웹 프로세스 재시작
   - 일반: `sp`
   - 필요 시 강제: `pkill -KILL -f "/home/pinky/pinky_web/pinky_web.py" && sp`

## 검증 명령 (항상 실행)
```bash
tail -n 30 /tmp/pinky_sp/web.log | grep "camera flip mode"
pgrep -fa '^python3 /home/pinky/pinky_web/pinky_web.py'
```

정상 기준:
- 로그에 설정값과 동일한 `camera flip mode: <value>` 출력
- 실행 인자 확인:
  - `none`이면 `--hflip --vflip`가 없어야 함
  - `hv`이면 `--hflip --vflip`가 있어야 함

## 중요한 사실
- 전원 재부팅은 필요 없다.
- 브라우저 강력 새로고침(`Ctrl+Shift+R`, `Ctrl+F5`)은 서버 실행 인자를 바꾸지 못한다.
- 실행 반영은 반드시 웹 프로세스 재시작으로 이루어진다.
- `pinky-mjpeg.service`(systemd system service)와 `sp`를 동시에 쓰면 충돌한다.

## 자주 헷갈리는 포인트
- `sb`는 `.bashrc` 변경 시 필요하고, `.env`만 바꿨다면 핵심은 `sp`다.
- `pgrep -af`는 간혹 자기 자신 커맨드를 함께 보여 혼동될 수 있다.
  - 위 검증 명령처럼 `^python3 /home/pinky/pinky_web/pinky_web.py` 정규식으로 확인한다.

## 운영 원칙
- 카메라 방향 문제는 먼저 `none`으로 원위치 확인 후 필요한 경우 `hv` 적용.
- 임시 하드코딩/강제 lock 코드는 넣지 않고 `.env` 한 곳만 유지한다.

## 서비스 충돌 방지 (필수 1회)
- 아래 서비스를 끄고 `sp`만 사용한다:
```bash
sudo systemctl disable --now pinky-mjpeg.service
```
- 확인:
```bash
systemctl is-active pinky-mjpeg.service
```
`inactive`여야 정상이다.
