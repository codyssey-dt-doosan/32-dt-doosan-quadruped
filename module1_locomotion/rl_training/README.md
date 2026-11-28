# rl_training

담당: **도훈**

박스 모델 Go2(다리 0.18 + 0.10 m, 13.2 kg, ±30 N·m)의 PPO 보행 정책을 Genesis에서 학습한다(보너스 #8, 옵트인). ROS 패키지가 아니다(`COLCON_IGNORE`). 학습한 정책은 `mpc_controller`의 `rl_node`가 Gazebo에서 numpy로 돌린다.

## 환경

ROS를 source하지 않은 터미널에서 Python 3.12 venv를 쓴다(Genesis 1.5 + rsl-rl 6.0 + torch 2.15, Apple Metal 또는 CUDA).

```bash
uv pip install -r requirements.txt
```

## 정합 (Sim2Sim)

관절 순서·기본 자세·Kp/Kd·액션 스케일·토크 한계·주기(dt 0.005 × 4 = 50 Hz)·관측 구성·DR 범위·관절 armature(Genesis 기본 0.1을 Gazebo에 맞춰 0으로 고정)·중력(Genesis 기본 9.81 대신 Gazebo 월드 9.8)은 `mpc_controller/config/sim2sim.json` 한 파일에서 읽고, 코드에 숫자를 다시 적지 않는다(아래 값은 설명용).

| 파일 | 내용 |
|------|------|
| `assets/go2_box.urdf` | `mpc_controller/rl_asset.py`가 SDF에서 생성(직접 고치지 않는다) |
| `cfgs.py` | sim2sim.json → env·관측·보상·명령 설정, PPO 설정(예제값) |
| `go2_box_env.py` | Genesis 평지 환경(예제 `go2_env.py` 기반, Apache-2.0) |

- 행동: q_des = q_nom + action_scale·clip(a)를 URDF 관절 한계로 자른다(`rl_node`와 같은 식). PD는 Genesis `control_dofs_position`(±tau_max)
- 지연: env마다 0~4 물리 스텝(0~20 ms) 동안 직전 목표를 유지한다
- DR(reset마다 env별): 마찰 0.2~1.2, base 질량 −1~+2 kg, kp·kd ×0.8~1.2. `get_cfgs(s2s, dr=False)`는 공칭값(mu 0.8, 지연 0)
- 관측: actor `policy` 45차원(각속도, 중력, 명령 (v_x, 0, w_z), q − q_nom, q̇, 직전 행동), critic은 `privileged`(참값 선속도, 마찰, 추가 질량)를 더 본다
- 종료: 기울기 > 45°, base_link 지면 접촉, 20 s, Genesis 오류 env

## Colab 학습

`colab_train.ipynb`(Colab Pro, T4/L4/A100). RL 코드가 GitHub에 push되어 있어야 한다.

| 셀 | 내용 |
|----|------|
| 1~2 | `BRANCH` 지정, `nvidia-smi`, Drive 마운트 |
| 3 | 속도 측정용 핀 설치(`genesis-world`·`rsl-rl-lib`·`tensorboard`, Colab 기본 torch 사용). push 전에도 실행 가능 |
| 4 | T4 속도 측정: Genesis v1.5.0 공식 `go2_train.py`(-b 4096, 101 iter)의 총 시간·step/s(`Steps per second`·`Time elapsed` 마지막 줄). 비교 기준은 Mac 약 5.7만 step/s. 실측(2026-11-27): 무료 T4는 백엔드 할당 실패, **L4 21.2만 step/s**(101 iter 53 s, Mac의 약 3.7배) |
| 5 | `BRANCH` 클론, `requirements.txt` 설치(`torch==` 줄 제외) 후 `train.py` 실행. `MAX_ITER`는 총 반복 수로, 가장 큰 `model_N.pt`를 빼 남은 만큼만 학습한다. `--log-dir`은 Drive `rl_go2`, `--save-interval 50`마다 체크포인트 저장. run 폴더에 `model_*.pt`가 있으면 `--resume`, 없으면 새로 시작 |

- Pro 한도: 백그라운드 실행 없음, 세션 최대 약 12 h(그 전에 끊길 수 있음), 동시 세션 1개. 브라우저 탭을 열어 두고, Mac은 `caffeinate -dims`로 잠자기를 막는다
- 끊기면 노트북을 다시 열어 `BRANCH` 셀, Drive 마운트, 설치(3), 변수 셀(`EXP`·`MAX_ITER`·`LOG_DIR`), 학습 셀을 차례로 실행하면 Drive의 마지막 체크포인트에서 이어 간다(`BRANCH`·변수 셀을 건너뛰면 `NameError`)
- `--resume`은 run 시작 때 남긴 `sim2sim.json`과 `provenance.json`의 학습 코드 sha256(`go2_box_env.py`·`cfgs.py`·`train.py`·`export.py`·URDF·`requirements.txt`)이 지금과 같을 때만 이어 간다(HEAD·미커밋 여부는 기록만 한다). 재연결 사이에 push로 이 파일이 바뀌면 거부한다 — `provenance.json`의 커밋으로 맞추거나 새 `EXP`로 시작한다
- 학습 쪽(resume·`export.py`·`eval.py`)은 `sim2sim.json` 전체를 엄격히 비교한다. 그래서 중력 키가 생기기 전의 run(`flat_v1`~`v3`)은 이제 eval·export가 `gravity` 차이로 거부된다(커밋된 `flat_v3.npz`는 `rl_node`에서 경고만 내고 돈다)
- 지형 run(`--terrain`)을 resume하면 env별 지형 레벨이 체크포인트에 없어 평균 1.5로 돌아간다(정책·std·lr은 이어진다). 재개 직후 약 42 iter는 잘린 첫 에피소드만 집계돼 보상·에피소드 길이가 0 근처로 떨어졌다가 회복한다(로깅 착시)

## 평가

```bash
python eval.py --run-dir DIR --ckpt model_N.pt          # G1 평지: 4조합 × 15 s, DR 없음. 오차 <10%·전도 0이면 PASS
python eval_terrain.py --run-dir DIR --ckpt model_N.pt  # 시험값 난이도(정보용): 계단 0.10 m·경사 20°·파이프 0.08 m, v 0.3 × 20 s
python export.py --run-dir DIR --ckpt model_N.pt --out ../mpc_controller/policies/NAME.npz
python eval_slope.py --run-dir DIR --ckpt model_N.pt [--deg 20 15]  # Genesis 경사 관문(Gazebo 전): 마지막 줄 GATE PASS/FAIL, 종료 코드 0/1
python train.py -e NEW --terrain --init-from RUN/model_N.pt  # 파인튜닝: 새 run에 actor·critic 가중치만, optimizer는 새로(첫 lr 1e-5), iter 0부터
```

- `eval_slope.py`: tile 6.65 m 경사(20°·15°)·평지에서 출발 yaw 거울쌍 {0, ±2.5, ±5, ±10°}, 명령 (0.3, 0, 0), DR 없음. 경사 띠 위 평균 회전 ω·피치 오차(내리막 = 평균 피치 − 경사각, 오르막 = 평균 피치 + 경사각, 기수 내림이 +)를 run마다 잰다. 관문: 경사 |mean ω| ≤ 0.004·max |ω| ≤ 0.008, 내리막 모든 run |피치 오차| ≤ 3°, 평지 max |ω| ≤ 0.004. 띠에 못 들어간 run은 NaN으로 남아 FAIL이다. 타일 끝 이탈(체비셰프 r ≥ 반폭 − 0.1)은 전도로 세지 않는다
- `eval_terrain.py`·`eval_slope.py`는 롤아웃 직전 `torch.manual_seed(0)`으로 시드한다. 그 전에는 env 생성 때 쓰는 난수 하나가 늘어도 결과가 바뀌었다(rough_v2 브랜치에서 model_2800 계단 오르기 완주가 3/8 → 4/8, 물리는 같음). 같은 체크포인트는 시드 뒤 코드에서 다시 잰 값끼리 비교한다
- `--init-from`은 `--resume`과 같이 쓸 수 없고, `--init-optimizer`는 `--init-from`과만 쓴다. 출처(run·ckpt·sha256·optimizer)는 `provenance.json`과 npz meta의 `init_from`에 남는다

- `eval_terrain.py`: 학습 타일 생성기(`terrain.py`)를 rows 2로 써서 난이도 1.0 행이 시험값이 되게 한다. 타입(계단 오르기·내리기, 경사 오르기·내리기, 파이프)마다 8 env를 출발 xy ±0.3 m·yaw ±10°로 흔든다(DR이 없으면 같은 타일 env가 똑같이 움직인다). 타일 가운데에서의 체비셰프 진행으로 3단 상당(1.4 m)·타일 끝(2.45 m) 도달과 전도를 센다. 타일 계단은 6단이라 Gazebo 코스(3단)보다 길다 — G3를 대신하지 않는다
- `eval.py`는 32 env가 같은 조건이라 사실상 1회 시행이다. 이웃 체크포인트끼리 PASS·FAIL이 오가는 것도 그 탓으로 보인다

### rough_v1 (P3 험지, 2026-11-28)

- 학습: 코드 `7ce73ca`, `--terrain`, 4096 env, 3000 iter, Colab T4 5.0~5.5만 step/s. VM이 두 번 회수돼 `model_450`·`model_1450`에서 재개했다(위 resume 항목의 레벨 초기화를 두 번 겪었다). 마지막 평균 지형 레벨은 4.97이다
- 체크포인트 선택: 2500~3000을 50 간격으로 G1에 넣었다. PASS는 2500·2550·2700·2750·2800·2850·2950, FAIL은 2600(w 18.2%)·2650(후진 15.0%)·2900(후진 11.3%)·3000이다. 후보는 `eval_terrain.py`로 비교했다

| ckpt | G1 (최대 오차) | 시험값 전도 /40 | 3단 상당 /40 | 계단 오르기 6단 완주 /8 | 나머지 4지형 완주 /32 |
|---|---|---|---|---|---|
| 3000 | FAIL ((0,0.5) w 10.2%) | 3 | 36 | 0 | 31 |
| **2800** | PASS (8.2%) | **0** | **40** | 3 | **32** |
| 2750 | PASS (7.0%) | 4 | 39 | 2 | 28 |
| 2700 | PASS (8.6%) | 0 | 38 | 3 | 28 |
| 2550 | PASS (3.5%) | 0 | 39 | 0 | 32 |

- `model_2800`을 `../mpc_controller/policies/rough_v1.npz`로 export했다. Gazebo 결과는 `mpc_controller` README에 있다(G2 GATE 12/12, G3 PASS 8/30·전도 0 대 MPC 5/30·13)
- G3 RL 실패 원인(코드·실측 대조, 궤적 기록 전이라 추정 포함):
  - ① **경사·요철에서 헤딩이 돈다.** 회전 속도는 0.03~0.06 rad/s이고, 평지 G2는 −0.002다.
    - 20° 1회차는 40 s에 +74° 돌았다. 측정 기록의 최대 |y| 값이 그 헤딩으로 벽(±1.75 m)에 닿는 위치와 1 cm 안에서 맞는다(6회 중 4회).
    - 20° 1·3회차는 회전 속도 0.034·0.031, 최대 |y| 1.424·1.420 m로 같아, 무작위가 아니라 재현되는 현상이다.
    - 후보 원인: 회전 추종 보상(가중 0.2, sigma 0.0625)으로는 0.03 rad/s 오차의 보상 손실이 0.3%뿐이다. 학습 IMU 잡음 σ 0.05가 이 편향보다 크고, 회전 0을 오래 유지하는 명령이 드물다.
  - ② **내리막에서 몸이 지형보다 5~7° 덜 숙는다.** 헤딩과 무관하다. 곧게 간 15° 3회차도 −5.06°였다.
  - ③ **파이프를 넘는 동안 0.11~0.22 m/s로 느려진다.**
  - 배포 쪽(중력 투영 부호·프레임, 경사로 형상)은 학습과 같음을 코드로 확인했다. ①이 정책 고유인지 Sim2Sim 차이인지는 Genesis에서 횡 이동·헤딩을 재야 가린다. 지금 `eval_terrain.py`는 진행만 잰다
  - 정정(rough_v2 설계 때 확인): ②를 "중력 기준 수평 페널티 때문"이라고 적은 적이 있으나 틀렸다. rough_v1 보상 10개에는 몸 기울기 항이 아예 없다. Genesis `eval_slope.py`에서도 20° 내리막 ω +0.067~+0.086 rad/s·피치 오차 −5~−7°로 재현돼 정책 고유다

### rough_v2·rough_v3 (파인튜닝, 2026-11-28)

model_2800에서 ①·②·③을 잡으려고 파인튜닝했다(설계·계획: vault `DT-작업노트-2026-11-28-RL-rough_v2-설계`·`-계획`).

- 변경: 회전 추종 `tracking_ang_vel` 0.2 → 0.5, 재표본 명령의 30%는 wz = 0, rsl-rl `Symmetry`(좌우 거울 증강 + 거울 손실, `mirror.py`), 지형 법선 자세 보상 `terrain_orientation`(1 − (n·z_b)², n은 base xy ±0.25 m 중앙차분), 파이프 열 2개(8열), 시작 레벨 상한 3 → 6, `--init-from`(가중치만, 새 Adam 첫 lr 1e-5)
- 자세 보상은 "판정에 맞춘 보상"이다. G3 경사 판정(지형과 평행 ±5°)에 맞추려고 넣었다
- 운영: Colab L4, 스모크(151 iter·1024 env) → 같은 VM에서 본 학습(1500 iter·4096 env, 약 5.5만 step/s, 45분). 두 run 모두 VM 회수 없이 끝났고, 스모크·본 학습 4회에 약 5.6 CU를 썼다

| run | 출발 | 자세 보상 | 거울 손실 계수 | 스모크 | 본 학습 마지막 레벨·거울 손실 |
|---|---|---|---|---|---|
| rough_v2 | rough_v1 model_2800 | −5 | 1 | 안정(6항목 PASS) | 5.65 · 6e-4 |
| rough_v3 | rough_v2 model_1050 | −20 | 10 | 안정(6항목 PASS) | 5.44 · 2e-4 |

- Genesis 경사 관문(`eval_slope.py`): rough_v2·rough_v3 모두 1000~1500(50 간격) 22개가 FAIL이다
  - ① 헤딩 편향은 크게 줄었다. 20° 내리막 평균 ω가 +0.0795(model_2800)에서 v2 1000~1100은 ±0.001~0.002 수준이 됐다. 남은 평균 ω 실패는 체크포인트마다 부호가 바뀌는 잔여 좌우 비대칭이다(추론 때 정확히 대칭화하면 0). max |ω| 실패는 대부분 ±5·±10°로 출발한 run이 경사 아래로 정렬하는 대칭 동작이다
  - ② 내리막 피치 오차는 v2 −4.0~−6.0°, v3 −3.5~−5.5°로 ±3°를 넘었다. 평지 피치는 v2 −1.2~−2.6°에서 v3 −0.1~−0.2°로 고쳐졌다
  - 원인 조사: 보상 법선과 진단 피치는 같은 상태에서 0.04° 안에서 일치해 정의·측정 버그가 아니다. 물리적으로 필요한 자세도 아니다 — rough_v2 학습 중간의 model_500은 20° 내리막을 −0.8°·0.26 m/s로, 같은 보상에서 더 높은 점수로 걸었다. 학습이 iter 700~1000을 지나며 몸통을 중력 수평 쪽 3~4°에 두는 걸음새로 굳었고, 자세 가중을 4배로 올려도 경사 오프셋은 약 20%만 줄었다. 붙잡는 압력은 경사 밖 학습 분포(계단 추정)에 있으나 출처는 가르지 못했다
  - 내리막 수평 속도는 model_2800 0.24~0.26 m/s에서 v2·v3 0.29~0.33으로 명령(0.3)에 가까워졌다
- 체크포인트 선택(관문 폴백): 미리 정한 기준(실패 항목 수 → 초과분 합)으로는 rough_v2 model_1050(실패 4)이 1위였다. 도훈이 결과를 본 뒤 G3 경사 판정에 직접 걸리는 피치를 우선해 **rough_v3 model_1150**(내리막 피치 오차 −3.5·−3.7°로 최소, 실패 6)을 골랐다. 결과를 본 뒤의 기준 변경이다

| ckpt | 관문 실패 | 20°·15° 내리막 피치 오차 | G1 (최대 오차) | eval_terrain 전도 /40 | 3단 상당 /40 |
|---|---|---|---|---|---|
| rough_v2 model_1050 | 4 | −4.9 · −5.3° | PASS (3.9%) | 0 | 37 |
| **rough_v3 model_1150** | 6 | **−3.5 · −3.7°** | PASS (6.0%) | 2 (계단 내리기) | 35 |

- `model_1150`을 `../mpc_controller/policies/rough_v3.npz`로 export했다(meta `init_from` = rough_v2 model_1050). 경사 자세 대신 계단이 약해졌다(G3에서 계단 전도 위험). Gazebo 결과는 `mpc_controller` README에 있다

## 테스트

```bash
python -m pytest tests/ -q
```

CPU 백엔드, 약 15 s. ROS 쪽 정합 테스트는 `mpc_controller/test/test_sim2sim.py`·`test_rl_asset.py`.

- `test_env_sim2sim.py`: dof 이름 매핑, PD·force range·dt·중력·지연, 관절 +0.1 rad 때 발 이동 방향 = `leg_kinematics` FK, 관측 배치, DR 표본 범위
- `test_obs_cross.py`: `rl_node`의 관측·행동 식(`mpc_controller/rl_obs.py`, numpy)이 env `obs_buf`·`_q_des`와 1e-5 안으로 같다
- `test_open_loop.py`: Gazebo가 보낸 q_des 10틱(`tests/data/`)을 개루프로 재생한 관절 RMS가 틱 0~5 <0.02·5~10 <0.08 rad. armature 0.1이면 실패하는 음성 대조 포함
- `test_eval_terrain.py`: 시험값 행의 형상(계단 단 높이·경사각·파이프 높이)과 완주·전도 latch(먼저 일어난 쪽만 남는다)
- `test_export.py`·`test_train_io.py`: numpy 정책 = torch actor, `sim2sim.json`이 바뀐 run의 export 거부, 학습 코드 해시 비교(`code_drift`, resume 거부 근거), 출처 meta(`train_git`·`init_from`), 로그 폴더 처리, `--init-from` 가중치만·optimizer 새로·첫 lr 1e-5·`--resume`/`--init-optimizer` 조합 거부
- `test_mirror.py`: 거울 순열·부호(FL↔FR·RL↔RR, hip·vy·wz·roll), mirror(mirror(x)) = x, 증강 배치, Genesis 물리 대칭(거울 행동 10스텝 뒤 관측이 거울, 최대 차 9e-8)
- `test_rough_v2_cfg.py`: 보상·명령·Symmetry 설정값, 구성 뒤 `runner.cfg` pickle, wz = 0 비율이 두 재표본 경로(reset·4 s)에서 0.3 ± 0.03이고 vx는 그대로
- `test_terrain_orientation.py`: 1 − (n·z_b)² 값, 20° 경사 법선의 각도·방향, 계단·border·heightfield 밖에서 유한한 단위 법선, plane이면 (0, 0, 1)
- `test_eval_slope.py`: 관문 규칙(평균만 초과·오르막 피치 비관문·평지 0.004·빈/NaN run FAIL), 전도·이탈 구분, yaw wrap
