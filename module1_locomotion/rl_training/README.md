# rl_training

담당: **도훈**

박스 모델 Go2(다리 0.18 + 0.10 m, 13.2 kg, ±30 N·m)의 PPO 보행 정책을 Genesis에서 학습한다(보너스 #8, 옵트인). ROS 패키지가 아니다(`COLCON_IGNORE`). 학습한 정책은 `mpc_controller`의 `rl_node`가 Gazebo에서 numpy로 돌린다.

## 환경

ROS를 source하지 않은 터미널에서 Python 3.12 venv를 쓴다(Genesis 1.5 + rsl-rl 6.0 + torch 2.15, Apple Metal 또는 CUDA).

```bash
uv pip install -r requirements.txt
```

## 정합 (Sim2Sim)

관절 순서·기본 자세·Kp/Kd·액션 스케일·토크 한계·주기(dt 0.005 × 4 = 50 Hz)·관측 구성·DR 범위·관절 armature(Genesis 기본 0.1을 Gazebo에 맞춰 0으로 고정)는 `mpc_controller/config/sim2sim.json` 한 파일에서 읽고, 코드에 숫자를 다시 적지 않는다(아래 값은 설명용).

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
| 4 | T4 속도 측정: Genesis v1.5.0 공식 `go2_train.py`(-b 4096, 101 iter)의 총 시간·step/s(`Steps per second`·`Time elapsed` 마지막 줄). 비교 기준은 Mac 약 5.7만 step/s |
| 5 | `BRANCH` 클론, `requirements.txt` 설치(`torch==` 줄 제외) 후 `train.py` 실행. `MAX_ITER`는 총 반복 수로, 가장 큰 `model_N.pt`를 빼 남은 만큼만 학습한다. `--log-dir`은 Drive `rl_go2`, `--save-interval 50`마다 체크포인트 저장. run 폴더에 `model_*.pt`가 있으면 `--resume`, 없으면 새로 시작 |

- Pro 한도: 백그라운드 실행 없음, 세션 최대 약 12 h(그 전에 끊길 수 있음), 동시 세션 1개. 브라우저 탭을 열어 두고, Mac은 `caffeinate -dims`로 잠자기를 막는다
- 끊기면 노트북을 다시 열어 `BRANCH` 셀, Drive 마운트, 설치(3), 변수 셀(`EXP`·`MAX_ITER`·`LOG_DIR`), 학습 셀을 차례로 실행하면 Drive의 마지막 체크포인트에서 이어 간다(`BRANCH`·변수 셀을 건너뛰면 `NameError`)

## 테스트

```bash
python -m pytest tests/ -q
```

`test_env_sim2sim.py`: dof 이름 매핑, PD·force range·dt·지연, 관절 +0.1 rad 때 발 이동 방향 = `leg_kinematics` FK, 관측 배치, DR 표본 범위(CPU 백엔드, 약 15 s). ROS 쪽 정합 테스트는 `mpc_controller/test/test_sim2sim.py`·`test_rl_asset.py`.
