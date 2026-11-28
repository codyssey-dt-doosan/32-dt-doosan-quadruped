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
```

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

## 테스트

```bash
python -m pytest tests/ -q
```

CPU 백엔드, 약 15 s. ROS 쪽 정합 테스트는 `mpc_controller/test/test_sim2sim.py`·`test_rl_asset.py`.

- `test_env_sim2sim.py`: dof 이름 매핑, PD·force range·dt·중력·지연, 관절 +0.1 rad 때 발 이동 방향 = `leg_kinematics` FK, 관측 배치, DR 표본 범위
- `test_obs_cross.py`: `rl_node`의 관측·행동 식(`mpc_controller/rl_obs.py`, numpy)이 env `obs_buf`·`_q_des`와 1e-5 안으로 같다
- `test_open_loop.py`: Gazebo가 보낸 q_des 10틱(`tests/data/`)을 개루프로 재생한 관절 RMS가 틱 0~5 <0.02·5~10 <0.08 rad. armature 0.1이면 실패하는 음성 대조 포함
- `test_eval_terrain.py`: 시험값 행의 형상(계단 단 높이·경사각·파이프 높이)과 완주·전도 latch(먼저 일어난 쪽만 남는다)
- `test_export.py`·`test_train_io.py`: numpy 정책 = torch actor, `sim2sim.json`이 바뀐 run의 export 거부, 학습 코드 해시 비교(`code_drift`, resume 거부 근거), 출처 meta(`train_git`), 로그 폴더 처리
