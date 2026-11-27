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

## 테스트

```bash
python -m pytest tests/ -q
```

CPU 백엔드, 약 15 s. ROS 쪽 정합 테스트는 `mpc_controller/test/test_sim2sim.py`·`test_rl_asset.py`.

- `test_env_sim2sim.py`: dof 이름 매핑, PD·force range·dt·중력·지연, 관절 +0.1 rad 때 발 이동 방향 = `leg_kinematics` FK, 관측 배치, DR 표본 범위
- `test_obs_cross.py`: `rl_node`의 관측·행동 식(`mpc_controller/rl_obs.py`, numpy)이 env `obs_buf`·`_q_des`와 1e-5 안으로 같다
- `test_open_loop.py`: Gazebo가 보낸 q_des 10틱(`tests/data/`)을 개루프로 재생한 관절 RMS가 틱 0~5 <0.02·5~10 <0.08 rad. armature 0.1이면 실패하는 음성 대조 포함
- `test_export.py`·`test_train_io.py`: numpy 정책 = torch actor, `sim2sim.json`이 바뀐 run의 export 거부, 학습 코드 해시 비교(`code_drift`, resume 거부 근거), 출처 meta(`train_git`), 로그 폴더 처리
