# CycloneDDS 네트워크 설정 가이드 (WireGuard wg0)

## 환경 구성

| 구성 요소 | 인터페이스 | IP 주소 |
|---|---|---|
| xr_teleoperate (텔레오퍼레이션) | `wg0` | `10.86.160.2` |
| unitree_sim_isaaclab (시뮬레이션) | `wg0` | `10.86.160.3` |

두 노드 모두 **DDS Domain ID = 1** (시뮬레이션 모드)을 사용한다.

---

## 1. 실행 명령어

### xr_teleoperate 쪽 (10.86.160.2)

```bash
cd xr_teleoperate/teleop/
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim --network-interface wg0
```

- `--sim` → DDS Domain ID를 `1`로 설정
- `--network-interface wg0` → CycloneDDS가 `wg0` 인터페이스를 사용

### unitree_sim_isaaclab 쪽 (10.86.160.3)

```bash
cd unitree_sim_isaaclab/
python sim_main.py --device cpu --enable_cameras --task <TASK> --enable_dex3_dds --robot_type g129 --network-interface wg0
```

- 시뮬레이션은 항상 Domain ID `1` 고정
- `--network-interface wg0` → CycloneDDS가 `wg0` 인터페이스를 사용

---

## 2. 내부 동작 원리

`--network-interface wg0`을 넘기면 내부적으로 다음과 같이 처리된다:

### unitree_sdk2_python의 ChannelFactory.Init() 흐름

```
ChannelFactoryInitialize(domain_id=1, networkInterface="wg0")
  └─ ChannelFactory.Init(id=1, networkInterface="wg0")
       └─ config = ChannelConfigHasInterface.replace('$__IF_NAME__$', 'wg0')
       └─ Domain(1, config)          # CycloneDDS 도메인 생성
       └─ DomainParticipant(1)       # 도메인 참가자 생성
```

이때 생성되는 CycloneDDS XML 설정:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="wg0" priority="default" multicast="default"/>
            </Interfaces>
        </General>
        <Tracing>
            <Verbosity>config</Verbosity>
            <OutputFile>/tmp/cdds.LOG</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

이 XML이 CycloneDDS에게 **wg0 인터페이스만 사용해서 DDS 통신하라**고 지정한다.

> 참고 파일: `unitree_sdk2_python/unitree_sdk2py/core/channel_config.py`

---

## 3. 대안: CYCLONEDDS_URI 환경변수로 직접 XML 설정

`--network-interface` 대신 CycloneDDS XML 파일을 직접 만들어서 환경변수로 지정할 수도 있다. 이 방법은 멀티캐스트 비활성화, peer 주소 직접 지정 등 세밀한 제어가 필요할 때 유용하다.

### 3-1. XML 설정 파일 작성

**Pro 6000 쪽** — `¬/custom/cyclonedds_teleop.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="wg0" priority="default" multicast="false"/>
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer Address="10.86.160.3"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Tracing>
            <Verbosity>warning</Verbosity>
            <OutputFile>/tmp/cdds_teleop.LOG</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

**G1 쪽** — `¬/custom/cyclonedds_sim.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="wg0" priority="default" multicast="false"/>
            </Interfaces>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer Address="10.86.160.2"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Tracing>
            <Verbosity>warning</Verbosity>
            <OutputFile>/tmp/cdds_sim.LOG</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

핵심 포인트:
- `multicast="false"` + `<AllowMulticast>false</AllowMulticast>`: WireGuard는 기본적으로 멀티캐스트를 지원하지 않으므로 비활성화
- `<Peer Address="..."/>`: 상대방 IP를 직접 지정하여 유니캐스트 discovery 수행

### 3-2. 환경변수 설정 후 실행

```bash
# xr_teleoperate 쪽 (10.86.160.2)
export CYCLONEDDS_URI="file:///home/eunwoo/Projects/custom/cyclonedds_teleop.xml"
cd xr_teleoperate/teleop/
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim
# --network-interface는 생략 (CYCLONEDDS_URI가 우선)

# unitree_sim_isaaclab 쪽 (10.86.160.3)
export CYCLONEDDS_URI="file:///home/eunwoo/Projects/custom/cyclonedds_sim.xml"
cd unitree_sim_isaaclab/
python sim_main.py --device cpu --enable_cameras --task <TASK> --enable_dex3_dds --robot_type g129
# --network-interface는 생략
```

> `sim_main.py` 12번째 줄에 주석 처리된 `os.environ["CYCLONEDDS_URI"]`를 활성화해서 코드 내에서 직접 설정할 수도 있다.

---

## 4. 방법 비교: --network-interface vs CYCLONEDDS_URI

| 항목 | `--network-interface wg0` | `CYCLONEDDS_URI` (XML) |
|---|---|---|
| 설정 난이도 | 간단 (CLI 인자 하나) | XML 파일 작성 필요 |
| 멀티캐스트 제어 | 불가 (기본값 사용) | 가능 (`AllowMulticast`) |
| Peer 직접 지정 | 불가 | 가능 (`<Peers>`) |
| WireGuard 호환성 | 멀티캐스트 문제 가능 | 완전 호환 |
| 추천 상황 | 같은 LAN, 빠른 테스트 | WireGuard/VPN 환경 |

**WireGuard(wg0) 환경에서는 XML 방식(방법 3)을 권장한다.** WireGuard 터널은 멀티캐스트를 지원하지 않는 경우가 많아서, `--network-interface wg0`만으로는 DDS discovery가 실패할 수 있다.

---

## 5. 통신 확인 및 디버깅

### wg0 인터페이스 상태 확인

```bash
# 양쪽 모두
ip addr show wg0
ping 10.86.160.3   # teleop → sim
ping 10.86.160.2   # sim → teleop
```

### CycloneDDS 로그 확인

```bash
# 로그 파일 위치 (XML에서 설정한 경로)
cat /tmp/cdds.LOG           # --network-interface 방식
cat /tmp/cdds_teleop.LOG    # XML 방식 (teleop)
cat /tmp/cdds_sim.LOG       # XML 방식 (sim)
```

로그에서 확인할 것:
- `using network interface wg0` — 올바른 인터페이스 사용 여부
- `new participant` — 상대방 DDS participant 발견 여부
- `error` / `failed` — 오류 메시지

### DDS 토픽 통신 확인

양쪽을 실행한 후, sim 쪽 콘솔에서 다음 메시지가 출력되면 연결 성공:
```
[DDSManager] DDS system initialized (domain=1, interface=wg0)
```

teleop 쪽에서 `r` 키를 눌러 텔레오퍼레이션을 시작하면 sim 쪽 로봇이 움직여야 한다.

---

## 6. 주요 DDS 토픽 (참고)

```
xr_teleoperate (10.86.160.2)              unitree_sim_isaaclab (10.86.160.3)
────────────────────────                   ────────────────────────────────
  PUBLISH  rt/lowcmd            ──►        SUBSCRIBE  rt/lowcmd
  PUBLISH  rt/dex3/left/cmd     ──►        SUBSCRIBE  rt/dex3/left/cmd
  PUBLISH  rt/dex3/right/cmd    ──►        SUBSCRIBE  rt/dex3/right/cmd

  SUBSCRIBE rt/lowstate         ◄──        PUBLISH    rt/lowstate
  SUBSCRIBE rt/dex3/left/state  ◄──        PUBLISH    rt/dex3/left/state
  SUBSCRIBE rt/dex3/right/state ◄──        PUBLISH    rt/dex3/right/state
```

양쪽 모두 **같은 Domain ID (1)** 과 **같은 네트워크 인터페이스 (wg0)** 를 사용해야 서로의 토픽을 발견하고 통신할 수 있다.
