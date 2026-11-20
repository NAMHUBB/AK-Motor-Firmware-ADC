# stm-gripper-controller (Clicker 4 STM32F407VG / AK 시리즈 Robotiq 브리지)

`../temp/ak80-9-firmware`의 Robotiq RTU ↔ CubeMars AK MIT 프로토콜 매핑 펌웨어를 STM32F407 보드로 옮긴 소스다. 2026-11-14부터 대상 보드는 **MikroE Clicker 4 for STM32(STM32F407VG)** 이고, 핀은 `../stm-gripper-261119`과 같다. 프로토콜·제어 로직(`src/lib.rs` 이하 모듈)은 원본과 같고, 보드 종속 부분(`main.rs`, `modbus.rs`, `ad7175.rs`)만 보드 핀맵에 맞췄다. 폴더 이름의 Mango는 처음 이식 대상이던 Mango-M32F4 보드에서 온 것이다.  CSP+CST 자동 전환과 레지스터 맵은 [HYBRID_CSP_CST_ko.md](HYBRID_CSP_CST_ko.md)를 참고한다.

- **ADC 차동 + CH1 게인 + D3000(2026-11-20)**: 한 펌웨어에서 모터와 ADC가 함께 되도록, ADC를 CANopen 펌웨어(`../stm-clicker4-adc-can-485-261119`)와 같게 맞췄다. 모터 쪽 코드는 그대로다.
  - 측정: **ADCn = AIN(2n) − AIN(2n+1)** 차동(채널 레지스터 0x8001, 0x8043, 0x8085, 0x80C7, 0x8109, 0x814B), **내부 2.5 V 기준 + 입력 버퍼 켬**(SETUPCON0 0x1320, `adc::SETUPCON_MEASURE`). 기준이 내부라 Click의 REF 점퍼(기본 4.096 V)와 상관없다. ADC1에는 CANopen 펌웨어와 같은 식으로 **× 0.87038**(`adc::CH_GAIN`)을 곱한다. ADC 블록 버전은 2 → **1**(차동)이고, 콘솔은 칸에 `AIN0/AIN1`로 표시한다. 하루 전의 GND 기준(버전 2)은 브리지의 공통 전압(약 2.5 V)을 재서 ADC0이 범위 초과였다.
  - 레지스터(겹치지 않게 분리): 모터는 Robotiq **D1000~D1007**(명령)·**D2000~D2007**(상태)와 0x1000~0x14FF, ADC는 0x1500 블록과 **D3000~D3005**(새로 추가, FC03/FC04 읽기 전용, 두 슬레이브 주소 모두)다. D3000 = ADC0 mV, D3001 = ADC1 mV(i16), D3002/D3003 = ADC0 µV, D3004/D3005 = ADC1 µV(i32, 상위 워드 먼저), CANopen 펌웨어의 D3000 블록과 같은 배치다. CANopen 펌웨어가 ADC를 두던 D2010·D2020은 이 펌웨어의 모터 상태 D2000 자리라 겹쳤다.
  - 자가 점검: 핀 스캔(GND 기준)은 버퍼를 끈 0x1020으로 따로 재고, 채널 점검은 `AIN(2n) - AIN(2n+1)`로 적는다. 차동은 음수도 정상이라 "BELOW GND" 판정을 쓰지 않는다(`adc::pair_verdict`).
  - 시험 132개(기본·`--no-default-features`), clippy `-D warnings`, rustfmt 통과. 빌드 `cargo build --release --locked` → `output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc-diff-d3000/`(81,172바이트, HEX sha256 `8d43e98b…`, 검증 통과). 보드(UID 0035005b)에 `probe-rs download --verify`로 올렸다. 이전 소스는 `backup/before_adc_differential_261120/`.
- **ADC 입력을 GND 기준으로 변경, 아날로그 전원 진단(2026-11-19)**: 스트레인 게이지 앰프 출력은 GND 기준 한 가닥이다. 그래서 **ADCn = AIN(2n) − GND(REF−)**, 즉 ADC0 = AIN0, ADC1 = AIN2로 바꿨다. 채널 레지스터는 0x8016, 0x8056, 0x8096, 0x80D6, 0x8116, 0x8156이다. 이전에는 차동 AIN(2n)/AIN(2n+1)이라, AIN0·AIN2만 연결된 보드에서 열린 AIN1·AIN3 때문에 약 −1.18 V가 흘렀다. GND 근처를 재므로 **입력 버퍼를 껐다**(SETUPCON0 0x1300 → 0x1000, 내부 기준 0x1320 → 0x1020). 버퍼를 켜면 입력이 AVSS + 0.1 V 이상이어야 하고, 끄면 AVSS − 50 mV까지 된다. AVDD1/5 점검만 예전 0x1320을 쓴다. Modbus ADC 블록 버전은 1 → **2**(GND 기준)다. 콘솔(`../../UI/AK-Motor-Console-ADC`)은 1·2를 모두 받고 칸에 `AIN0 vs GND`로 표시한다. 시작 점검에 세 가지를 더했다: 온도 센서(정상이면 상온 약 140 mV), 내부 2.5 V 기준으로 잰 REF± 핀, **AIN0~AIN16 핀 스캔**(GND 기준, 핀마다 약 0.1초). 판정 "TOO LARGE"는 "BELOW GND"(−50 mV 아래)로 바꿨다. 빌드: `cargo build --release --locked` → `output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc-gnd/`(81,028바이트). 시험 130개(기본·`--no-default-features`), clippy `-D warnings`·rustfmt 통과.
  - **보드(UID 0035005b)에서 본 결과: ADC 4 Click의 아날로그 쪽에 전원이 없다.** SPI·ID 0x3CDE·변환 횟수는 정상이라 3.3 V(디지털)는 들어온다. 그러나 AVDD1이 0.001 V로 재지고(정상 약 5 V), 온도 센서는 −0.29 mV(= −273 °C, 정상 약 140 mV)다. 아무것도 연결되지 않은 AIN4~AIN16이 모두 같은 −25.7 mV에 잡음 ±2 µV로 멈춰 있고(전원이 있으면 떠서 흔들린다), AIN0을 자기 자신과 잰 값도 0.618 mV에 잡음이 0이다. AIN0 −32.2 mV, AIN2 −33.7 mV, AIN1·AIN3은 +2.5 V(범위 초과)였다. 그래서 지금까지의 스트레인 값(이 펌웨어와 `stm-gripper-261119` 모두)은 의미가 없다. Click의 5 V 핀(mikroBUS 5V–GND)을 계측기로 재고 5 V를 넣은 뒤 다시 본다. 이전 소스는 `backup/before_adc_single_ended_261119/`.
- **ADC 시작 자가 점검(2026-11-14)**: "ADC0·ADC1이 켜지는데 측정이 제대로 안 됨"(값이 0.1초 사이에 수 mV씩 튀고, ADC0이 −695 mV에 머무는 등)을 가려내려고 넣었다. 펌웨어의 AD7175-8 설정(채널 0x8001·0x8043…, SETUPCON0 0x1300, FILTCON0 0x060E, ADCMODE 0xA000, IFMODE 0x0040)과 읽는 순서는 클리커4 펌웨어와 같으므로, 원인은 기준 전압·입력 배선·브리지 여기 전압 같은 하드웨어 쪽이다. Click을 찾으면 한 번(약 2초) 아래를 재고 `cargo run` 로그에 `ADC check:` 줄로 남긴다.
  1. AVDD1: (AVDD1−AVSS)/5 입력을 내부 2.5 V 기준으로 잰다.
  2. REF+/REF− 핀 전압: AVDD1 기준, 입력 버퍼 끔. 클리커4 설정(SETUPCON0 0x1300)은 내부 기준이 아니라 **REF± 핀의 외부 기준**으로 잰다(클리커4 주석의 "internal 2.5 V"는 값과 다름). 그래서 REF± 핀에 2.5 V가 없으면 모든 값이 엉터리다. REF±가 2.4~2.6 V면 클리커4 설정 그대로 쓰고, 아니면 **내부 2.5 V 기준(0x1320)으로 바꿔 재며** 경고를 남긴다.
  3. 0점: AIN0끼리 잰 값과 흔들림으로, ADC 자체 잡음이다.
  4. 켜진 채널마다 0.3초 평균과 흔들림(최대−최소). 판정은 다음과 같다(`adc::verdict`).
     - OK
     - 범위 초과: ±2.5 V 밖
     - **UNSTABLE**: 1 mV 넘게 흔들림. 입력이 떠 있다.
     - **TOO LARGE**: ±100 mV 넘게 안정. 브리지 출력(수 mV)이 아니다. 한쪽이 열렸거나 전원에 붙었다.
  - 경고 문구에 확인할 핀(AINn/AINn+1), 브리지 여기 전압, Click과의 GND 공통이 나온다. 브리지 출력 +는 AIN(2n), −는 AIN(2n+1)에 연결한다.
  - 시험: 130개(새 시험 3개: 점검 입력 코드·REF/AVDD 환산·판정) × 4가지 조합, clippy·rustfmt 통과. **보드에서는 아직 돌려 보지 않았다**(ST-Link를 다른 `cargo run`이 쓰고 있었음). 이전 소스는 `backup/before_adc_self_check_261114/`.
- **`cargo run --release` 로그에 ADC0~5 mV(2026-11-14)**: 스트레인 게이지 측정용으로 100 ms마다 한 줄씩 나온다. 예: `ADC mV | ADC0 1.688 | ADC1 -7.183 | ADC2 off | …`. 값은 µV를 그대로 mV 소수 셋째 자리로 찍는다(부호 유지, -5 µV = `-0.005`). 꺼진 채널은 `off`, 켜졌지만 아직 변환 전이면 `--`, 범위를 넘으면 값 뒤에 `OVR`가 붙는다. 이전의 1초마다 µV 디버그 로그는 없앴다.
  - **채널 고르기**: 부팅 때 켜지는 채널은 빌드 때 `ADC_MASK`로 정한다. `ADC_MASK=0x3f cargo run --release`면 6채널 모두, 안 주면 ADC0·ADC1(0x03)이다. 16진수(`0x..`)나 10진수, 0~0x3F이고 범위 밖이면 빌드가 실패한다. 콘솔의 −/+(FC06 0x1502)로도 실행 중에 바꿀 수 있다.
  - **RTT 비차단**: `defmt-rtt`에 `disable-blocking-mode`를 켜고 버퍼를 4 KiB로 늘렸다(`.cargo/config.toml`의 `DEFMT_RTT_BUFFER_SIZE`). probe-rs는 RTT를 차단 모드로 바꾸므로, 전에는 `cargo run`을 Ctrl-C로 끝낸 뒤 버퍼가 차면 펌웨어 전체(Modbus·CAN 포함)가 멈출 수 있었다. 이제는 호스트가 없으면 로그만 버려진다. Ctrl-C 뒤 30초가 지나도 ADC 변환이 계속되는 것을 확인했다.
  - **CAN 접속이 부팅을 막지 않음**: bxCAN은 버스에서 recessive 11비트를 봐야 동작을 시작한다. 전에는 `main`이 이것을 기다려서, 버스가 dominant로 붙잡혀 있으면(배선 오류) 부팅 로그와 ADC가 아예 시작되지 않았다. 이제 기다림은 CAN 태스크 안에서 하고, 2초 뒤 한 번, 그다음 30초마다 `CAN: bus not idle for N s (RX held dominant: …) Motors are off; ADC and Modbus keep running` 경고를 남긴다. 그래서 **CAN을 쓰지 않고 ADC와 Modbus만 써도 된다**(모터 채널은 피드백 없음으로 보인다).
  - 시험: 127개(새 시험 2개: mV 표기, `ADC_MASK` 해석) × 기본·`--no-default-features`·`gripper-homing`·`close-positive`, clippy·rustfmt 통과. 보드(UID 0035005b)에서 `cargo run --release`로 53줄/약 5초를 확인했다. `ADC_MASK=0x3f`에서는 6채널 값이 모두 나왔다. 이전 소스는 `backup/before_adc_mv_log_261114/`.
- **Mango 보드 → Clicker 4 for STM32 / STM32F407VG(2026-11-14)**: 이 펌웨어가 도는 보드는 Mango-M32F4가 아니라 클리커4 CST 펌웨어가 돌던 **Clicker 4 for STM32(STM32F407VG, 100핀, Flash 1 MB)** 다. 보드에 연결된 칩의 Flash 크기 레지스터가 1024 KB였고, 원래 들어 있던 펌웨어가 `stm-clicker4_CST`(`stm32f407vg`) 빌드였다. 그래서 보드 종속 부분을 모두 클리커4 프로젝트에 맞췄다.
  - 칩·메모리: `stm32f407ze`(512K) → **`stm32f407vg`**. `memory.x` Flash 1024K / RAM 128K(SRAM1+SRAM2, CCM 64K는 쓰지 않음, 클리커4와 같음). probe-rs 칩 이름 `STM32F407VG`(`.cargo/config.toml`, 아래 명령). `tools/export_firmware.py`·`verify_firmware.py`의 Flash 범위는 0x0810_0000까지.
  - RS-485: USART2 **PA2 TX / PA3 RX, PC15 = DE**(HIGH 송신 / LOW 수신), **PA4 LOW = /RE**, DMA1 스트림 6(TX)·5(RX). Mango의 USART6·PG8·PE5는 이 칩(100핀)에 없다.
  - CAN: CAN1 PB8 RX / PB9 TX(같음) + **PE7 LOW = CAN 트랜시버 STB 해제**(High면 대기 모드라 버스에 아무것도 나가지 않음).
  - ADC 4 Click: SPI3 PC10 SCK / PC11 MISO / PC12 MOSI / PD15 CS, DMA1 스트림 7(TX)·2(RX).
  - LED: PC8(클리커4 LED1)이 0.5초마다 깜박인다. Mango의 PF6은 없다.
  - 보드에 올림(2026-11-14, 보드 UID 0035005b 34335115 38313438): `output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc/`(74,912바이트, HEX sha256 `a1d9d76d…`)를 `probe-rs download --verify`로 굽고 되읽어 같음을 확인했다. 부팅 로그 `Clicker 4 STM32F407VG / AK40-10 + AK60-39 + ADC 4 Click … 2 motor(s)`, `AD7175-8 ID 0x3cde`(Click 검출), ADC0·ADC1 변환 중. Mac의 USB-RS485(`/dev/cu.usbserial-130`, 115200)로 Slave 1·2에 FC04 0x1500·0x1300 읽기 200회 모두 응답, 브로드캐스트 FC06 0x1502로 ADC 마스크가 바뀌는 것도 확인했다. CAN(모터)은 확인하지 않았다.
  - **주의 · 리셋 방법(2026-11-14 정정)**: `--connect-under-reset`은 이 ST-Link 연결에서 동작하지 않는다(`Timeout while attaching to target under reset`, NRST가 연결되지 않은 것으로 보임). 앞서 "하드웨어 리셋 뒤 정상"이라 적은 시험은 출력을 숨긴 채 돌려 실제로는 리셋되지 않았으므로 근거가 없다. 관찰한 것만 적으면: 디버거 소프트 리셋(`probe-rs reset`) 뒤 Modbus 무응답 3/3, 그 뒤 다른 디버거 접속에서 보드가 다시 시작되며 응답했다. `cargo run` 뒤(같은 날 CAN 배선을 바꾼 뒤)에는 USART2에 바이트가 전혀 들어오지 않았다(요청 154회, RX DMA 0바이트): RS-485 배선·전원을 확인할 일이다. 굽은 뒤 Modbus가 응답하지 않으면 보드 리셋 버튼이나 전원을 껐다 켠다.
  - 그 전 플래시(클리커4 CST 펌웨어, 205,392바이트)는 `backup/board_uid_0035005b_before_stm32f407_adc_261114/`에 1 MB 전체 HEX·BIN으로 있다. 이전 소스는 `backup/before_stm32f407_261114/`(Mango, ZE)와 `backup/before_clicker4_pins_261114/`(ZG로 바꿨던 중간본, 보드에 올린 적 없음).
- **스트레인 게이지 ADC0~5(ADC 4 Click, 2026-11-14)**: `../stm-gripper-261119`의 AD7175-8(MikroE ADC 4 Click) 측정을 옮겼다. 채널·설정은 클리커4와 같다: ADCn = 차동 +AIN(2n) / −AIN(2n+1), 양극성 ±2.5 V(SETUPCON0 0x1300, 외부 REF, 2.5 V로 환산), sinc5+sinc1 100 SPS, 켜진 채널을 차례로 변환(2채널이면 채널당 약 50 Hz, 6채널이면 약 17 Hz). 클리커4는 1 mV 단위(i16)였지만 브리지 출력이 수 mV라 **µV 단위 i32**로 낸다(24비트 한 칸 0.3 µV). 콘솔은 mV 소수 셋째 자리까지 보인다.
  - 결선(클리커4와 같음): **SPI3 PC10 = SCK, PC11 = MISO(Click SDO), PC12 = MOSI(Click SDI), PD15 = CS**, 3.3 V·GND(mikroBUS는 5 V도 주므로 Click이 5 V를 쓰는 구성이면 5 V도). DMA1 스트림 7(TX)·2(RX), 채널 0(USART6는 DMA2라 겹치지 않음). SPI3는 APB1(42 MHz)이라 42 / 32 = 1.3 MHz(클리커4는 5.25 MHz, 100 SPS에는 느려도 충분). MISO 풀업(Click이 없으면 ID 0xFFFF로 보임).
  - Modbus 0x1500(어느 모터의 Slave 주소로든, FC03/FC04 20워드): 0 매직 0x5347, 1 버전 1, 2 켜진 채널 마스크(비트 n = ADCn), 3 마스크를 바꾼 뒤 변환된 채널, 4 범위 초과 채널, 5 상태(0 시작, 1 동작, 2 AD7175-8 없음), 6 ID 레지스터, 7 변환 횟수(순환), 8~19 ADC0~5 µV(i32, 상위 워드 먼저, 꺼진 채널은 0). **FC06 0x1502 = 켜질 채널 마스크(0~0x3F), 부팅 기본 0x03(ADC0·ADC1)**. 모두 끄면(0) AD7175-8을 대기 모드로 둔다. ID는 1초마다 다시 읽어 Click이 빠지면 상태 2로 바뀌고 값이 0이 된다.
  - 소스: `src/adc.rs`(레지스터 값·환산·Modbus 블록, PC에서 시험), `src/ad7175.rs`(SPI3 드라이버 태스크). 모터 쪽 레지스터와 상태 버전(8)은 그대로라 기존 콘솔도 이 펌웨어와 동작한다. 콘솔은 `../../UI/AK-Motor-Console-ADC`.
  - 빌드: `cargo build --release --locked --features dual-motor` → `output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc/`(74,912바이트). 2026-11-14 Clicker 4 보드에 올렸다(위 항목).
  - 시험: 125개 통과(기본·`dual-motor`·`dual-motor gripper-homing`·`dual-motor close-positive`), `--release --features dual-motor` clippy `-D warnings`·rustfmt 통과. 새 시험 `src/adc_tests.rs` 8개: 채널 레지스터가 클리커4 값(0x8001…0x814B)과 같음, 코드→µV 환산(±2.5 V, 1 mV = 3356코드), 상태 바이트의 채널·범위 초과, AD7175-8 ID 판별, 마스크·기본값, 레지스터 배치, FC03/04/06 처리, 두 Slave 주소 모두에서 응답.
  - 못 한 것: 실물 ADC 4 Click 연결(배선, Click 전원·REF 구성, 노이즈). 이전 소스는 `backup/before_adc_261114/`.
- **AK60-39 V3.0 KV80 추가(2026-11-13)**: 모터마다 고급 설정 0x1200 워드 12로 모델을 고른다: **2 = AK40-10**(부팅 기본), **3 = AK60-39**. 0·1·4 이상이면 그 쓰기 전체를 무시한다. 모델은 **토크 OFF(rACT 0)에서만** 바뀐다: 켜져 있을 때 다른 모델을 쓰면 쓰기를 무시하고, 그래도 켜진 채 바뀌면 0x0D(세부 12)로 멈춘다(Kt가 11배 다르므로). Robotiq 블록 쓰기는 고른 모델을 그대로 둔다. 상태 0x1300 워드 14 비트 8~9가 모델이다.
  - AK60-39 값(CubeMars 사양표): 48 V, 39:1, 14극쌍, 정격 24 N·m / 피크 72 N·m, 정격 70 RPM(무부하 98 RPM, 48 V 기준), 정격 4.5 A / 피크 17 A. Kt는 AK40-10처럼 정격 토크 / 정격 전류 = 5.33 N·m/A(사양표의 모터 쪽 0.12 × 39 = 4.68보다 전류를 약 12 % 적게 보냄). Speed 1000 = 70 RPM, Force는 0~9 N·m 비율 그대로라 AK60-39에서는 9 N·m(정격의 37.5 %, 1.69 A)까지다. 24 V로 돌리면 실제 최고 속도는 약 절반이다.
  - 빌드: `cargo build --release --locked --features dual-motor` → `output/firmware-dual-motor-ak40-10-ak60-39/`(67,852바이트, HEX sha256 `22650474…`). 세 번째 보드(UID 00380054 52345003 20343734)에 굽고 67,852바이트를 되읽어 같음을 확인했다. 부팅 로그 `AK40-10 + AK60-39 Robotiq RTU bridge · 2 motor(s) · boot motor model 2`, Slave 1 → CAN ID 1, Slave 2 → CAN ID 2. 그 전 플래시(Windows에서 만든 172,025바이트 이미지, 이 폴더 출력물과 다름)는 `backup/board_uid_00380054_before_ak60_39_261113/`에 HEX·BIN으로 있다. 이전 소스는 `backup/before_ak60_39_261113/`.
  - 시험: 117개 통과(기본·`dual-motor`·`dual-motor gripper-homing`·`dual-motor close-positive`), `--release --features dual-motor` clippy `-D warnings`·rustfmt 통과. 새 시험 `src/ak60_tests.rs`: 모델은 OFF에서만 바뀜, 켜진 채 바뀌면 폴트, 위치 모드 9 N·m·70 RPM 램프, 속도·프로파일 패킷(546 ERPM/RPM), AK60-39 Kt로 과전류 판정, 피드백 속도 환산.
  - 못 한 것: 실물 AK60-39(회전 방향, 피드백 각도가 출력축 기준인지, 마찰·게인). 콘솔 기본 게인 Kp 10 / Kd 1은 추정값이다.
- **Activate 없음 · 모든 모드(2026-10-31)**: 활성화 원점 잡기를 기본 빌드에서 뺐다(`--features gripper-homing`일 때만 그리퍼 원점 잡기). 활성화는 첫 모터 피드백만 기다리고 바로 끝나며, 모터는 ±8회전 끝점 사이에서 그대로 움직인다(0~1024 = −2880~+2880°). 실행(rACT + rGTO)을 OFF에서 한 번에 보내도 활성화가 끝난 다음 tick을 rGTO 0→1로 보므로 CSP+CST도 CSP 단계부터 시작한다(이전에는 CST로 넘어가지 않았음). 모드 변경은 실행 중(rGTO 1이 이어질 때)에만 거부하고, 켜져서 멈춰 있는(위치 유지) 동안에는 어느 모드로든 바꾼다(0x1200 쓰기도 rGTO 1일 때만 거부). 콘솔 `../AK-Motor-Console-test`가 이에 맞춰 Activate 없이 Run·모드 선택을 한다.
  - 빌드: `cargo build --release --locked --features dual-motor` → `output/firmware-dual-motor-ak40-10-no-activate/`(66,260바이트, HEX sha256 `02b72a0e…`). Right 보드(UID 003a0047 52345003 20343734)에 굽고 66,260바이트를 되읽어 같음을 확인했다(부팅 로그 `AK40-10 Robotiq RTU bridge · 2 motor(s)`, Slave 1 → CAN ID 1, Slave 2 → CAN ID 2). 그 전 플래시(AK40-10 전용 빌드)는 `backup/right_board_before_no_activate_261031/`.
  - 시험: 106개 통과(기본·`dual-motor`·`dual-motor gripper-homing`·`dual-motor close-positive`), clippy `-D warnings`·rustfmt 통과. 새 시험: 멈춘 모터가 5개 모드 설정을 모두 받고 다음 rGTO에서 그 모드로 도는 것, OFF에서 보낸 실행이 활성화 뒤 CSP 단계로 시작하는 것(수정 전 소스로는 실패 확인). 이전 소스는 `backup/before_no_activate_261031/`.
- **AK40-10 전용(2026-10-31)**: AK60-6·AK80-9 프로필과 그 모델들만 쓰던 MIT 모드·MIT 패킷을 지웠다. 모터는 AK40-10 하나뿐이라 `ak40-10` 빌드 기능도 없어졌다(늘 AK40-10, Kp 1 / Kd 0.05로 부팅). Modbus 레지스터 배치는 그대로다: 설정 0x1200의 워드 12(모터 모델)는 **2만** 받고 0·1·3이면 그 쓰기 전체를 무시하며, 모드 1(MIT)도 무시한다. 워드 3·4(예전 MIT 게인)는 저장·되읽기만 하고 쓰지 않는다. 폴트 세부 13(MIT 범위)은 더 나오지 않는다. Speed 1000 = 370 RPM, Force는 0~9 N·m 비율 그대로이되 4.1 N·m에서 자른다. Right 회전 배율은 1.0(실제 출력 각도)으로 고정.
  - 빌드: `cargo build --release --locked --features dual-motor` → `output/firmware-dual-motor-ak40-10-only/`(66,356바이트, HEX sha256 `b68f27a9…`). Slave 1 → CAN ID 1(Left), Slave 2 → CAN ID 2(Right).
  - 시험: 104개 통과(기본·`dual-motor`·`dual-motor close-positive`), clippy `-D warnings`·rustfmt 통과. MIT·AK60-6·AK80-9 전용 시험 5개는 지우고, 나머지는 AK40-10 값(Kt 1.3/2.7, 14극쌍 × 10:1, 4.1 N·m)으로 바꿨다. 모델 워드 0·1·3과 모드 1을 거부하는지 새로 시험한다.
  - 보드에 올림: Right 보드(UID 003a0047 52345003 20343734)에 굽고 66,356바이트를 되읽어 같음을 확인했다. 부팅 로그 `AK40-10 Robotiq RTU bridge · 2 motor(s)`, `Modbus slave 1 -> CAN ID 1`, `Modbus slave 2 -> CAN ID 2`. 그 전 플래시(아래 듀얼 모터 AK40-10 빌드)는 `backup/right_board_before_ak40_10_only_261031/`, 그 전 소스는 `backup/before_ak40_10_only_261031/`.
- **AK40-10 듀얼 모터 + 원점 잡기(2026-10-31)**: `../stm-gripper-Mango-UR10e-V2` 펌웨어 소스(상태 버전 8)를 가져오고 이 폴더의 `MOTOR_CAN_ID`를 다시 얹었다. UR10e용 URScript·Modbus TCP 브리지는 넣지 않았다(호스트는 커스텀 ROS2 그리퍼 노드, RS-485 Modbus RTU 직접).
  - 빌드(이때, AK40-10 전용 이전): `cargo build --release --locked --features "dual-motor ak40-10"` → `output/firmware-dual-motor-ak40-10/`(HEX sha256 `57578ea9…`). 보드 1장이 Slave 1 → CAN ID 1(Left), Slave 2 → CAN ID 2(Right)를 맡고, 두 모터 모두 AK40-10으로 부팅한다(Kp 1 / Kd 0.05). 모터 ID는 R-LINK로 Left 1, Right 2. AK40-10은 MIT가 없어 위치 루프 토크를 서보 전류 명령(0x101 / 0x102)으로 보내고, Force는 9 N·m 비율 그대로이되 피크 4.1 N·m에서 자르며, Speed 1000 = 370 RPM이다.
  - Activate = 원점 잡기: 닫힘 끝까지 닫아 100 %, 계산한 이동량만큼 열어 0 %, 완전 열림에서 토크 OFF·활성 유지로 끝난다(gSTA 3, 최대 30 s). 그동안 들어온 rGTO는 거부하고, 끝난 뒤에도 호스트가 한 번 0으로 내려야 다시 받는다. 0~255(0x1000 블록은 0~1024) = 완전 열림~완전 닫힘. 목표 도착(gOBJ 3) 뒤 토크 OFF, 물체를 잡으면(gOBJ 2/1) 계속 누른다.
  - 호스트(ROS2 노드) 연동: RS-485 115200 8N1 포트 하나에 Slave 1·2를 번갈아 묻는다(보드 1장이 두 주소에 응답, 0 = 두 모터 동시 쓰기·응답 없음). 쓰기는 FC16(또는 FC23)만, 개수·주소가 맞지 않으면 응답 없이 무시한다(FC06 없음). 슬레이브마다 1 s 안에 한 번 이상 요청이 없으면 토크 OFF(0x09). 레지스터는 둘 중 하나: ① Robotiq 2F 블록(쓰기 0x03E8~, 읽기 0x07D0~, 3개 등 부분 접근 가능) — 쓰면 MCU 위치 모드, ② 0x1000~0x1003(action 0/1/9, position 0~1024, speed 0~1000, force 0~1000, 4개 한 번에) + 설정 0x1200~0x120F(16개 한 번에, 모드 6 = CSP+CST, 워드 12 모터 모델 **2**) + 상태 0x1100~0x110F. CSP+CST로 활성화한 뒤 ① 블록에 쓰면 0x0D(세부 12)로 멈춘다.
  - 게인 주의: 부팅 게인 Kp 1이면 마찰 0.05 N·m에서 Right(3 mm 리드, 반 칸 0.046 rad)가 목표 밖에 멈춰 도착(gOBJ 3)을 보고하지 못한다(시뮬레이션). 노드가 0x1200에 Kp 200 / Kd 100(2.00 / 0.100, AK60-6 그리퍼가 쓰던 값)을 쓰게 한다.
  - AK40-10에 맞춘 것: Right 회전 배율. AK60-6 Right는 보고 각도가 실제의 1/1.5라 1.5를 썼고, ak40-10 빌드는 실제 출력 각도를 본다고 보고 1.0이다(같은 11.25 mm 열림 = 1350°, 원점 잡기 6.70 rad/s, Left는 그대로 2686° / 10.05 rad/s). 열림 끝이 계산보다 가까우면 원점 잡기가 그 멈춤을 열림 끝으로 잡는다. 회전 방향은 양(+) 회전 = 열림이고, 반대로 달았으면 `--features "dual-motor ak40-10 close-positive"`.
  - 시험: 109개 통과(기본·`dual-motor`·`ak40-10`·`dual-motor ak40-10`·`dual-motor close-positive`), clippy `-D warnings` 통과. 새 시험: ID 1·2 AK40-10 각각 활성화 → Position 128(Speed 500 = 185 RPM으로 움직이는지 확인) → 물체 파지(Force 한도 안, CST 0.25 N·m 유지) → 완전 열림 토크 OFF, Force 1000이 4.1 N·m에서 잘리는 것. 이전부터 실패하던 `ak40-10` 시험 5개는 AK60-6 값을 기대하던 것이라, 부팅 모델의 한도를 쓰거나 AK60-6 설정을 명시하도록 고쳤다. 모든 시험이 보내는 CAN 프레임이 그 모터 ID로만 가는지도 확인한다.
  - 보드에 올림(2026-10-31): Right 보드(UID 003a0047 52345003 20343734)에 `probe-rs download --verify`로 굽고, 이미지 72,436바이트를 되읽어 같음을 확인했다. 부팅 로그: `2 motor(s) · boot motor model 2`, `Modbus slave 1 -> CAN ID 1`, `Modbus slave 2 -> CAN ID 2`. 그 전 플래시(01:01에 `probe-rs run`으로 올린 이전 빌드)는 `backup/right_board_before_dual_motor_ak40_10_261031/`에 BIN·HEX로 있다.
  - 못 한 것: 실물 AK40-10 그리퍼(모터 응답, 회전 방향, Right 회전 배율, 마찰·게인), ROS2 노드와의 실제 통신(노드 소스는 보지 못함). 이전 소스·문서는 `backup/before_ur10e_port_261031/`. 원래 UR10e용 레지스터 설명은 `../stm-gripper-Mango-UR10e-V2/UR10E_URSCRIPT_ko.md`.
- MCU: STM32F407VG(Clicker 4 for STM32, LQFP100, Flash 1 MB / SRAM 128 KB + CCM 64 KB), HSE 25 MHz → SYSCLK 168 MHz. `memory.x` Flash 1024K / RAM 128K. 2026-11-13까지는 Mango-M32F4(STM32F407ZE, 512K) 기준이었다.
- RS-485: USART2 PA2(TX)/PA3(RX), DMA1_CH6(TX)/DMA1_CH5(RX), PC15 = DE(HIGH 송신 / LOW 수신), PA4 LOW = /RE. (2026-11-13까지 Mango: USART6 PC6/PC7, PG8 DE, PE5 MUX.)
- (Mango 보드 기준) J9 결선: 1 = B(−), 2 = A(+), 3 = GND. J10 = 120 Ω 종단.
- (Mango 보드 기준) J11 결선: 1 = CANH, 2 = CANL, 3 = GND. J12 = 120 Ω 종단. Mango 보드에서는 CANH·CANL을 바꿔 연결해야 통신이 됐다(2026-11-14).
- Modbus: 115200 / 8N1 / Slave 9 (Robotiq 기본값). 빌드할 때 `MODBUS_SLAVE_ID=1~247`로 보드마다 다른 주소를 넣을 수 있다(범위 밖이면 빌드 실패). 부팅 로그에 주소가 찍힌다.
- 그리퍼 2개: 보드 두 장이 RS-485 한 줄을 같이 쓰고 Slave ID로 구분한다(왼쪽 1 / 오른쪽 2, `output/firmware-left-slave1`, `output/firmware-right-slave2`). 보드마다 RS-485 포트(J9)가 하나뿐이어도 된다: 보드가 신호를 넘겨주는 구조가 아니라 A끼리·B끼리·GND끼리 병렬로 묶는다. 한 보드의 J9 단자에 두 가닥(PC 쪽 + 다른 보드 쪽)을 함께 물리거나, 보드 옆 단자대에서 나눠 각 J9로 짧게 간다. 주소가 다른 보드는 응답하지 않아 DE(PG8)가 수신 상태로 남고, 브로드캐스트(주소 0) 쓰기에는 어느 보드도 응답하지 않으므로 충돌이 없다. CAN은 보드마다 따로라 두 모터 모두 ID 104 그대로 쓴다. J12(CAN 종단)는 두 보드 모두, J10(RS-485 종단)은 한 보드에만 끼운다. 콘솔 Sync는 펌웨어 변경 없이 동작한다: 실행 명령을 주소 0 브로드캐스트(FC16 0x1000)로 보내면 두 보드가 응답 없이 같은 프레임을 적용하고, 콘솔이 각 보드 상태로 확인한다. 콘솔은 두 그리퍼를 번갈아 조회한다(각 보드의 1 s 호스트 타임아웃 안쪽).
- 보드 1장 + 모터 2개(`--features dual-motor`, `output/firmware-dual-motor`): Slave 1 → CAN ID 1(Left), Slave 2 → CAN ID 2(Right). 모터 CAN ID는 R-LINK로 1·2로 바꿔 둔다. 한 보드가 두 주소에 각각 응답하고, 브로드캐스트(주소 0) 쓰기는 두 모터에 모두 적용한다(응답 없음). 모터마다 상태 머신·호스트 타임아웃·피드백 타임아웃·폴트가 따로라 한 모터만 빠지면 그 모터만 0x09로 멈춘다. CAN 버스 오류와 송신 큐 가득 참은 버스 공통 문제라 두 모터를 함께 멈춘다. 1 ms마다 두 모터에 한 프레임씩 보낸다. 콘솔은 그대로 Slave 1·2로 접속하면 되고 Sync도 같다. CAN 종단은 J12와 모터 쪽 단자대 끝에 120 Ω씩(전원 OFF에서 CANH–CANL 약 60 Ω). `MODBUS_SLAVE_ID`와 같이 쓰면 빌드가 실패한다.
- CAN: CAN1 PB8(RX)/PB9(TX), PE7 LOW = 트랜시버 STB 해제(Clicker 4), 1 Mbit/s, 모터 ID 104 (0x68). dual-motor 빌드는 1·2.
- **ID2 단일 모터 시험(2026-10-30)**: 모터 CAN ID도 빌드할 때 `MOTOR_CAN_ID=1~127`로 정한다(기본 104, dual-motor와 함께 쓰면 빌드 실패). `MODBUS_SLAVE_ID=2 MOTOR_CAN_ID=2 cargo build --release --locked` → `output/firmware-id2-position-test/`(Slave 2 → CAN ID 2, 상태 버전 7). Right 보드(UID 003a0047 52345003 20343734)에 올렸고, 이전 플래시는 `backup/right_board_before_id2_position_test_261030/`에 있다. 상태 버전 7이라 이전 콘솔(`../AK-Motor-Console`, Right 탭)의 MCU Position 모드로 시험한다. ID 2로 빌드한 상태에서는 프레임 ID를 0x68로 적어 둔 테스트 6개가 실패한다(정상: 프레임이 ID 2로 간다). Left 모터 단일 시험은 `MODBUS_SLAVE_ID=1 MOTOR_CAN_ID=1` → `output/firmware-id1-position-test/`(Slave 1 → CAN ID 1, 이전 콘솔 Left 탭). 같은 Right 보드에 올렸고 그 전(ID2 시험) 플래시는 `backup/right_board_before_id1_position_test_261030/`에 있다. **모터 하나씩 시험할 때는 dual-motor 빌드가 가장 간단하다**: `cargo build --release --locked --features dual-motor` → `output/firmware-dual-motor-position-test/`(Slave 1 → CAN 1, Slave 2 → CAN 2). 꽂힌 모터만 응답하므로 ID1은 Left 탭, ID2는 Right 탭으로 다시 굽지 않고 시험한다(콘솔 Left ID 1 / Right ID 2). Right 보드에 올렸고 그 전(ID1 시험) 플래시는 `backup/right_board_before_dual_motor_position_test_261030/`에 있다.
- **콘솔 v6 AK40-10 시험(2026-10-30)**: `../AK-Motor-Console-test`는 상태 버전 6(±4회전)만 받으므로, 8회전 이전 소스(`backup/before_8turns_261028/src`)를 `--features ak40-10`으로 빌드해 `output/firmware-console-v6-ak40-10/`에 두었다(Slave 9, CAN ID 104, AK40-10 부팅, Kp 1 / Kd 0.05). BIN/HEX는 2026-10-27 AK40-10 이미지와 같다(HEX sha256 `3f2b272e…`). 세 번째 보드(UID 00380054 52345003 20343734)에 올리고 되읽어 확인했으며, 이전 플래시(같은 펌웨어의 이전 디버그 빌드)는 `backup/board_uid_00380054_before_console_v6_ak40_261030/`에 BIN·HEX로 있다.
- CAN 모니터(Modbus 0x1400, 어느 모터의 Slave 주소로든 FC04 82워드): 보드가 받은 모든 CAN 프레임을 ID별로 최대 8개(횟수·마지막 데이터·경과 ms), 버스 오류 수와 마지막 종류, CAN 컨트롤러 TEC/REC·오류 상태를 보여 준다. 하드웨어 필터는 전부 받고, 모터 피드백은 소프트웨어에서 ID로 고른다. FC06 0x1400 = 1이면 ID 스캔: 서보 끄기(0x0F00|id, ID 1~255)와 MIT "exit motor mode"(표준 ID 1~127)만 1 ms에 한 ID씩 보내고 응답을 표에 모은다. 모든 모터가 OFF일 때만 돌고, 하나라도 켜지면 멈춘다. 2는 표와 카운터를 지운다. PC에서는 `python3 tools/can_monitor.py <포트> [--scan|--watch|--clear]`(콘솔은 닫고). 이전 dual-motor HEX는 `backup/before_can_monitor_261029/`에 있다.
- LED: PF6, 500 ms 주기 토글.
- 모터: AK40-10 V3.0 KV170(고급 설정 레지스터 12 = 2, 부팅 기본)과 AK60-39 V3.0 KV80(= 3, 2026-11-13부터). AK60-6(1)·AK80-9(0) 프로필은 지웠다.
- 부팅 기본 MCU 게인: Kp 1 / Kd 0.05 (이전 8 / 0.4, `backup/before_motor_gains_261027/`). 호스트가 설정을 쓰면 그 값으로 바뀐다.
- 이동 속도 0~1000 = 0~정격 속도(AK40-10 370 RPM, AK60-39 70 RPM).
- 위치 이동(MCU 위치·CSP+CST의 CSP 단계)에서 축이 부하에 막히거나 밀려나도 위치 기준점은 실측 위치보다 `1.1 × 토크 제한 / Kp` 이상 앞서지 않는다. 막혀 있는 동안의 토크와 접촉·CST 판정은 그대로이고, 부하가 풀리면 그 거리만 따라잡은 뒤 설정 속도로 목표까지 간다. 이전 소스와 HEX는 `backup/before_following_lag_261026/`에 있다.
- CSP+CST는 토크가 임계값 이상이고 출력축이 거의 정지(0.1 rad/s 이하)한 상태가 연속 초과 시간만큼 이어질 때만 CST로 전환한다(출발 가속 토크로는 전환하지 않음). CST 중 출력축 속도가 설정 속도(0이면 0.1 rad/s)를 넘으면 브레이크로 멈춘 뒤 CST로 다시 눌러 재파지한다. 이전 소스와 HEX는 `backup/before_cst_guard_261027/`에 있다.
- CSP+CST의 CST는 CSP가 밀던 방향(목표 위치 쪽)으로, CSP와 같은 토크 지령으로 계속 누르고, 설정한 CST 토크(고급 설정 8번)는 크기만 쓴다. 유지 중 gOBJ는 2/1이다. 이전 소스와 HEX는 `backup/before_cst_auto_direction_261027/`, `backup/before_hold_direction_261027/`에 있다.
- CSP+CST는 범위 끝(±2880°)을 넘지 않도록, 끝이 "멈춘 곳에서 최고 속도까지 간 거리"보다 가까워지면 모터의 전류 브레이크(공통 토크 제한)로 멈춘다. CSP는 그 자리에서 다시 출발하고 CST는 물체가 없는 것으로 보고 0x0D(세부 15)로 정지한다. AK60-6 + 볼나사 그리퍼의 파지력 환산은 [GRIP_FORCE_AK60-6_ko.md](GRIP_FORCE_AK60-6_ko.md)를 참고한다. 이전 소스와 HEX는 `backup/before_travel_guard_261027/`에 있다.
- 목표 위치 내부 단위 0~1024 = −2880~+2880° (±8회전, 총 16회전). 서보 피드백 한계(±3200°) 안쪽이다.
- UI와 Robotiq 레지스터(UR)는 위치·속도·토크를 0~255로 쓴다. 위치 한 칸 = 내부 4단위 = 22.5°(1/16회전)로 정확히 환산되어 **128 = 0°**, ±8칸 = ±180°, ±16칸 = ±1회전이다. 0 = −2880°, 255 = +2857.5°(+쪽 끝은 한 칸 짧음). 속도·토크는 `(값 × 1000 + 127) / 255`로 내부 0~1000이 된다.
- 토크는 모두 서보 전류 명령으로 보내고(AK40-10은 MIT 범위·토크 계수가 공개되지 않음), 피크 4.1 N·m에서 제한한다. Kt는 정격 토크 / 정격 전류(1.3 N·m / 2.7 A).

| 파일 | 내용 |
|---|---|
| `src/robotiq.rs`, `src/extended.rs`, `src/advanced.rs` | Robotiq / 확장 / 고급 Modbus 레지스터 처리 |
| `src/control.rs`, `src/bridge.rs` | 명령 해석, 상태 머신, CSP/CST 제어, 활성화 원점 잡기 |
| `src/cubemars.rs`, `src/config.rs` | AK 시리즈 CAN 패킷, 모터·동작·그리퍼(Left / Right) 설정 |
| `src/homing_tests.rs` | 원점 잡기·이동·파지 시뮬레이션 시험(AK40-10 ID 1·2 포함) |
| `src/ak60_tests.rs` | AK60-39(모델 3) 시험: 모델 전환 규칙, 토크·속도 환산 |
| `src/adc.rs`, `src/ad7175.rs`, `src/adc_tests.rs` | 스트레인 게이지 ADC0~5(ADC 4 Click, SPI3 PC10~12 / PD15), Modbus 0x1500 |
| `src/dispatch.rs` | 요청을 Slave 주소별 모터 채널로 나눔(브로드캐스트는 전체) |
| `src/monitor.rs`, `tools/can_monitor.py` | CAN 모니터·ID 스캔(0x1400), PC 판독 도구 |
| `src/can.rs`, `src/modbus.rs`, `src/transport.rs` | embassy CAN / RS-485 태스크, 모터 채널별 공유 상태 |

이식 전 EYOU CANopen 소스는 `backup/mango-eyou-original_261024/`에 있다. 모터 2개 지원 이전 소스는 `backup/before_dual_motor_261029/`에 있다(`ak40-10` 테스트 4개 실패는 그 전부터 있던 것: 기본 게인이 AK60-6 기준). 상태 버전은 8(고급 3, UR10e 펌웨어)이다. 7은 `backup/before_ur10e_port_261031/`(이전 콘솔 `../temp/ak80-9-control-ui`와 짝).

**2026-11-14부터 기본 빌드가 dual-motor다**(`Cargo.toml`의 `default = ["dual-motor"]`). 그래서 `cargo build --release`나 `cargo run --release`만 해도 Slave 1 → CAN ID 1, Slave 2 → CAN ID 2 펌웨어가 나온다. 그 전에는 기능 없이 빌드하면 모터 1개용(Slave 9)이 나와 콘솔이 Slave 1·2로 연결되지 않았다. 모터 1개용 보드는 `--no-default-features`를 붙인다(`MODBUS_SLAVE_ID`·`MOTOR_CAN_ID`는 그때만 쓸 수 있고, dual-motor와 같이 쓰면 빌드가 이유를 말하고 실패한다). `cargo run --release`는 굽고 RTT 로그를 보여 준다. Ctrl-C로 끝내도 보드는 계속 돌고, Modbus도 응답한다(시험함). `cargo run`의 러너에는 `--connect-under-reset`을 넣지 않는다: 이 ST-Link에서는 붙지 못하고 실패한다.

```bash
# 시험: 기본(dual-motor)과 모터 1개용
cargo test --lib --target aarch64-apple-darwin --locked
cargo test --lib --target aarch64-apple-darwin --locked --no-default-features
# 굽고 RTT 로그 보기(기본 = dual-motor, ADC mV 100 ms마다)
cargo run --release --locked
# 6채널 모두 켜고 로그 보기
ADC_MASK=0x3f cargo run --release --locked
cargo build --release --locked
python3 tools/export_firmware.py
python3 tools/verify_firmware.py --record output/firmware/artifact_check.json
# 그리퍼 2개: 보드마다 Slave ID를 넣어 빌드한다.
MODBUS_SLAVE_ID=1 cargo build --release --locked --no-default-features
python3 tools/export_firmware.py --out output/firmware-left-slave1
python3 tools/verify_firmware.py --folder output/firmware-left-slave1 --record output/firmware-left-slave1/artifact_check.json
MODBUS_SLAVE_ID=2 cargo build --release --locked --no-default-features
python3 tools/export_firmware.py --out output/firmware-right-slave2
python3 tools/verify_firmware.py --folder output/firmware-right-slave2 --record output/firmware-right-slave2/artifact_check.json
# 보드 1장 + AK40-10 2개 (Slave 1·2 → CAN ID 1·2, 원점 잡기)
cargo test --lib --target aarch64-apple-darwin --locked --features dual-motor
cargo build --release --locked --features dual-motor
python3 tools/export_firmware.py --out output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc
python3 tools/verify_firmware.py --folder output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc --record output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc/artifact_check.json
probe-rs download --chip STM32F407VG --binary-format hex --verify output/firmware-clicker4-stm32f407vg-dual-motor-ak60-39-adc/ak80_9_robotiq.hex
# 굽은 뒤 Modbus가 응답하지 않으면 보드 리셋 버튼이나 전원을 껐다 켠다(--connect-under-reset은 이 ST-Link에서 실패).
probe-rs reset --chip STM32F407VG --non-interactive
```
