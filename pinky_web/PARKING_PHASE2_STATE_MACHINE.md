# Parking Phase2 State Machine (Deprecated)

> Deprecated on 2026-03-04. Use `FINAL Integrated Technical Specification v3.0.md` as the active Phase 2 spec.

이 문서는 PINKY 주차장 2단계 주행의 확정 상태머신을 정의한다.

## State Order
1. `ENTRY`
2. `TURN_LEFT_1`
3. `MID_FORWARD_75` (75cm 직진)
4. `TURN_LEFT_2`
5. `ROW2_SCAN` (8번 면 전방 정지)
6. `SLOT_ALIGN_PARK` (좌회전 90도 + 후진 27cm 주차)

## State Definitions

### 1) ENTRY
- 목적: 게이트 통과 직후, 입구선 위 기준 자세에서 1열 전구간을 고정 시퀀스로 통과.
- 입력 상황:
  - 우측 주차면 일부(2,3,4번)가 보임.
  - 우측 벽면 녹색선은 PINKY 기준 약 35cm 거리로, 초기에는 신뢰 기준으로 약할 수 있음.
- 제어 기준:
  - 입구선 기준 정렬 자세에서 115cm 직진.
  - 이 구간은 기존 `ROW1_SCAN + ROW1_END_DETECT + ROW1_OFFSET_FORWARD`를 통합한 고정 구간으로 정의.
- 완료:
  - 115cm 직진 완료.
  - 전방 벽면까지 잔여 거리 약 400mm 수준 도달.
- 전이: `TURN_LEFT_1`.

### 2) TURN_LEFT_1
- 목적: 첫 번째 좌회전.
- 완료: 새로운 주행축으로 정렬.
- 전이: `MID_FORWARD_75`.

### 3) MID_FORWARD_75
- 목적: 75cm 직진.
- 완료: 75cm 누적 이동 완료.
- 전이: `TURN_LEFT_2`.

### 4) TURN_LEFT_2
- 목적: 두 번째 좌회전.
- 완료: 우측 2열(4면) 탐색 자세 확보.
- 전이: `ROW2_SCAN`.

### 5) ROW2_SCAN
- 목적: 우측 2열 슬롯 탐색 후 8번 면 전방 정지.
- 완료: 8번 면 앞 정지.
- 전이: `SLOT_ALIGN_PARK`.

### 6) SLOT_ALIGN_PARK
- 목적: 목표면 정렬 및 주차.
- 동작: 좌회전 90도 후 후진 27cm.
- 완료: 주차 완료 플래그 설정.

## Fixed Motion Constraints
- ENTRY는 1열 구간을 분리 판단하지 않고 고정 시퀀스로 처리.
- ENTRY 고정값: 115cm 직진, 종료 시 전방 잔여거리 약 400mm.
- 중간 구간 직진 75cm 고정.
- 최종 주차: 좌회전 90도 + 후진 27cm.

## Notes
- 벽/기둥 하단 녹색선(폭 2.3cm)을 경계 기준으로 사용.
- 흰색 주차선(폭 2.3cm)은 슬롯 구조 보조 기준으로 사용.
- `A/B 100cm` 정의: A 존 주차면 측과 B 존 주차면 측 사이의 측간 거리.
- 벽-벽 전체 거리는 `150cm`로 정의한다.
  - 근거: 측간 `100cm` + 양쪽 여유 `25cm x 2`.
  - 타일 기준: `50cm x 3칸`.
