# 인수인계: 차량 회전 궤적 검토 프로그램 (SweptPathChecker)

작성일: 2026-11-15 · 기준 커밋: `2c8cb20` (브랜치 `claude/cool-volta-dyrkw3`)

이 문서만 읽고 이어서 작업할 수 있도록 정리했습니다. 사용법은 [README.md](README.md),
최종 사용자용 요약은 [packaging/사용법.txt](packaging/사용법.txt)에 있습니다.

---

## 1. 한눈에 보기

| 항목 | 내용 |
|---|---|
| 목적 | 도면(DXF/DWG) 위에 차량 경로를 찍으면 회전 가능 여부(충돌, 여유폭, 목표 도달, 잭나이프)를 자동 판정 |
| 형태 | CAD와 무관한 **독립 실행 프로그램** (Python + Tkinter). CAD 버전과 관계없이 동작하고 결과는 DXF/PDF로 내보냄 |
| 위치 | 저장소 `danba1229/counting-program`의 `swept-path-checker/` 폴더 (같은 저장소의 통행량 카운터와는 무관) |
| 브랜치 | `claude/cool-volta-dyrkw3` — **main 에 아직 합쳐지지 않음** |
| 배포 | GitHub Actions(Windows 서버)가 푸시마다 exe·소스 묶음을 빌드하고 자체 점검까지 실행 |
| 상태 | 경유점 방식 판정, 대용량 도면, PDF 보고서까지 완료. 다음 큰 작업은 **자동 경로 탐색** (§9) |
| 검증 | 단위·GUI 테스트 42개, exe 자체 점검 8항목 (Windows Server 2025에서 통과) |

---

## 2. 배경과 결정 사항

- **배경**: 기존에 쓰던 상용 회전 궤적 플러그인이 특정 CAD 버전(2014)에서만 동작해서, 더 범용적인 도구가 필요했습니다.
  기존 프로그램 파일은 사용하거나 수정하지 않았고, 공개된 차량 운동학 이론으로 처음부터 새로 만들었습니다.
- **요구사항**: 도면 올리기 → 차량 유형 선택 → 출발·목적지 지정 → 회전 가능 여부 자동 판정
- **사용자와 합의한 결정**

| 주제 | 결정 |
|---|---|
| 저장 위치 | 별도 저장소 대신 현재 저장소의 독립 폴더 (세션 권한상 새 저장소 생성 불가). 폴더째 옮기면 그대로 동작 |
| 판정 방식 | **경유점 방식 먼저** (사용자가 경유점을 찍고 그 경로를 따라갈 수 있는지 검사). 자동 탐색은 다음 단계 |
| 차량 | 설계기준자동차(소형·대형·세미트레일러) + **미니버스(카운티 장축)** |
| 동작 | **전진 기본**, 후진과 여러 번 꺾기(전진↔후진 전환)도 지원 |
| 배포 | exe(PyInstaller) + exe 없이 실행하는 소스 묶음(`시작하기.pyw`) 두 가지 |

---

## 3. 기능 목록 (현재)

- **도면**
  - DXF는 빠른 판독기로 직접 읽음. DWG는 GstarCAD가 설치된 PC에서 COM 자동화로 DXF 변환 후 읽음 (`swept_path/gstarcad.py`), 없으면 무료 ODA File Converter
  - 블록, 중첩 블록, 배열 삽입, 해치 경계, 스플라인, 두께 있는 폴리선 지원
  - 단위 자동 인식 (정보가 없으면 선 길이로 mm/m 추정)
  - 변환 결과 캐시: 같은 파일을 다시 열면 1초 안쪽
- **장애물 레이어**
  - 목록에서 선택, 이름 검색, 도면에서 선을 클릭해 켜기/끄기
  - 레이어 설정 저장/불러오기 (JSON)
- **차량**: 기본 4종, 제원 편집, 사용자 차량 저장 (`user_vehicles.json`, exe 옆)
- **경로**
  - 출발점(드래그로 방향), 경유점, 목적지(드래그로 도착 방향)
  - 전진/후진 구간, 점 끌어서 이동, 방향 화살표 회전, 숫자 수정
  - 되돌리기(Ctrl+Z, 100단계), 경로 초기화
- **판정**: 충돌·여유폭 부족, 목표점/방향 도달 실패, 트레일러 잭나이프. 경로 이탈이 크거나 핸들을 끝까지 꺾은 상태로 오래 주행하면 주의로 표시
- **결과**
  - 차체 궤적, 여유폭 외곽, 바퀴·모서리 궤적, 차량 윤곽, 충돌 위치, 위치 슬라이더
  - 내보내기: **PDF 보고서**(그림 / 상세 / 충돌 확대도), PNG, 원본+결과 DXF, 결과만 DXF, txt, 작업 파일(json)
- **화면**
  - 도구 막대, 단계 표시(✓), 빈 화면 안내, 접히는 범례
  - 단축키: Ctrl+O 열기 / Ctrl+S 저장 / Ctrl+Enter 실행 / F 화면 맞춤
  - Windows 고해상도(DPI) 대응

---

## 4. 코드 구조

```
swept-path-checker/
├─ app.py               GUI 전체 (Tkinter). 화면 구성, 마우스 조작, 실행/저장 흐름
├─ selftest.py          exe 자체 점검 (--selftest). exe 옆에 selftest_result.txt 생성
├─ 시작하기.pyw          exe 없이 실행: 처음 한 번 .venv 생성·부품 설치 후 app.py 실행 (--check 로 점검)
├─ SweptPathChecker.spec PyInstaller 설정 (onedir, 창 모드, 샘플 도면 포함, PIL Tk 모듈 명시)
├─ swept_path/
│  ├─ vehicles.py       차량 제원, 기본 차량, 회전반경 → 조향각 변환, 사용자 차량 저장
│  ├─ geometry.py       차량 자세 → 차체 사각형, 바퀴·모서리 위치
│  ├─ simulate.py       경로 추종 주행 시뮬레이션 (전진/후진/세미트레일러)
│  ├─ analysis.py       충돌 검사(shapely STRtree), 궤적 외곽, 판정, 요약 문구
│  ├─ drawing.py        도면 읽기 진입점, 레이어·단위, ezdxf 방식 변환기, 캐시, 별도 프로세스 해석
│  ├─ fastdxf.py        빠른 DXF 판독기 (줄 단위 스트리밍 + numpy 계산)
│  ├─ render.py         도면을 이미지 한 장으로 그리기 (보이는 선만), 확대·이동 미리보기
│  ├─ export.py         DXF/txt 내보내기, 작업 파일 저장·불러오기
│  ├─ report.py         PDF/PNG 보고서 (A4 가로 200 dpi, 한글 글꼴 자동 탐색)
│  └─ ui_theme.py       색상·글꼴·ttk 스타일, 차량 평면도 그림
├─ tests/               pytest (test_engine.py: 계산·도면·보고서, test_gui_editing.py: 화면 조작)
├─ tools/               bench_view.py (화면 속도 측정), make_source_package.py (소스 묶음 생성)
├─ samples/             sample_parking.dxf + make_sample.py (생성 스크립트)
└─ packaging/사용법.txt  exe 묶음에 함께 들어가는 사용자 안내
```

저장소 루트의 `.github/workflows/build-swept-path-checker.yml`이 Windows 빌드 작업입니다.

### 데이터 흐름

```
DXF ─▶ drawing.load_drawing ─▶ (캐시 있음) npz 읽기
              │                 (없음, 8 MB↑) 별도 프로세스 → npz → 메인에서 읽기
              ▼
        fastdxf.parse_file ─▶ Converter.records ─▶ Drawing(verts, offsets, layer_index, bboxes)
        (실패 시 ezdxf 방식 _load_ezdxf 로 자동 전환)
              │
              ├─▶ render.DrawingRenderer ─▶ 화면 (PIL 이미지 1장)
              └─▶ polylines_m(장애물 레이어, 경로 주변 범위) ─▶ analysis.build_obstacles
                                                                      │
Scenario(출발·경유점) ─▶ simulate.Simulator.run ─▶ SimResult ─▶ analysis.analyze ─▶ Analysis
                                                                      │
                                         화면 표시 / report.export_pdf / export.export_dxf
```

- **좌표계**: 내부 계산은 m 단위입니다. 도면 좌표는 `unit_scale`(mm 도면이면 0.001)로 바꿉니다.
- **화면 변환**: `canvas = (x*s + tx, -y*s + ty)`
- **기준점**: 출발점은 앞축 중심입니다. 경로를 따라가는 점은 전진일 때 앞축, 후진일 때 뒷축, 세미트레일러 후진일 때 트레일러 축입니다.

---

## 5. 핵심 알고리즘과 조정값 (수정 시 주의)

| 부분 | 방식 | 값·위치 |
|---|---|---|
| 차량 모델 | 자전거 모델(뒷축 기준) + 세미트레일러 킹핀 운동학 | `simulate.py` `_integrate` |
| 최대 조향각 | 바깥 앞바퀴 회전반경 R → `Rr = √(R²−L²) − T/2`, `δ = atan(L/Rr)` | `vehicles.steer_from_turn_radius` |
| 전진 추종 | 앞축 기준 Pure Pursuit, `δ = atan2(k sinα, 1 + k cosα)`, `k = 2L/Ld` | 전방주시 `Ld = clip(0.8L, 2.5, 6.0)` |
| 후진(단일) | 뒷축 기준 Pure Pursuit | `Ld = clip(1.2L, 3, 8)` |
| 후진(세미) | 트레일러 축 Pure Pursuit → 목표 꺾임각 → 트랙터 조향 2단 제어 | `trailer_look_factor=1.0`, `gamma_gain=4.0` |
| 조향 속도 | 핸들 끝~끝 시간 + 주행 속도로 1 m당 조향 변화 제한 | 전진 8 km/h, 후진 4 km/h 기본 |
| 충돌 | 0.1 m 간격 차체 사각형(여유폭만큼 확대) ↔ 장애물 선 교차 | `CheckOptions.clearance=0.3` |

> ⚠ **세미트레일러 후진 이득값**은 여러 경로로 시험해 가장 안정적인 조합을 고른 것입니다. 범위가 좁아서 이득을 6 이상으로 올리면 불안정해집니다. 바꿀 때는 `tests/test_engine.py`의 세미트레일러 후진 테스트를 반드시 돌리세요.

### 도면 읽기에서 실제 도면으로 찾아 고친 문제 (회귀 테스트 있음)

1. **두께 있는 폴리선**: 중심선이 아니라 띠 모양 테두리로 읽어야 합니다. 빠른 경로에서 제외해 ezdxf로 보냅니다.
2. **크게 확대해 넣은 블록의 곡선**: 곡선 분할 허용오차를 삽입 배율로 나눠야 각지지 않습니다. 허용오차별로 블록을 캐시합니다.
3. **길이 0 구간이 섞인 해치**: ezdxf가 경계 전체를 버리므로, 깨진 조각만 건너뛰고 경계는 살립니다. `_hatch_boundaries`가 담당합니다.
4. **비균일 배율 블록 안의 회전된 블록**: ezdxf `virtual_entities`는 근사값을 냅니다. 이 프로그램은 행렬을 곱해 정확히 계산합니다.
5. **옛 형식 DXF의 유니코드 표기**(`\U+BCBD`): 레이어 이름을 디코딩합니다.

변환 방식을 바꾸면 `drawing.CACHE_VERSION`을 올려야 사용자 PC의 예전 캐시가 무효화됩니다(현재 5).

---

## 6. 개발 환경 · 실행 · 테스트 · 빌드

```bash
pip install -r requirements.txt pytest pyinstaller
python app.py                       # 실행
python app.py --selftest            # 자체 점검 (8항목)
python -m pytest -q                 # 테스트 42개 (GUI 테스트는 화면 없으면 건너뜀)
pyinstaller --noconfirm SweptPathChecker.spec   # dist/SweptPathChecker/
python tools/bench_view.py 도면.dxf  # 화면 반응 속도 측정
```

- **Linux 클라우드 환경에서 GUI를 시험할 때**
  - `Xvfb :99 &` 실행 후 `DISPLAY=:99`로 실행합니다.
  - Ubuntu 기본 Pillow에는 `ImageTk`가 없습니다. `python3.12 -m pip install --break-system-packages --ignore-installed pillow`로 다시 설치하세요.
  - tkinter는 `apt-get install python3-tk`로 설치합니다(python3.12 전용).
- **별도 프로세스 해석 주의**: Windows에서 쓰는 spawn 방식이라, 시험 스크립트에도 `if __name__ == "__main__":` 보호 구문이 필요합니다. 없으면 측정 스크립트가 무한 반복됩니다. 앱 본체에는 들어 있습니다.
- **변환 캐시 위치**: Windows는 `%LOCALAPPDATA%\SweptPathChecker\drawings`, Linux는 `~/.cache/SweptPathChecker/drawings`입니다. 최근 20개만 유지합니다. 테스트는 `conftest.py`가 임시 폴더로 바꿉니다.

### Windows 빌드 (GitHub Actions)

`swept-path-checker/**`가 바뀌어 푸시되면 다음 순서로 자동 실행됩니다.

1. 단위 테스트
2. 화면 속도 비교
3. exe 빌드
4. exe 자체 점검 (2번: 기본 폴더 / 다른 작업 폴더에서)
5. 소스 묶음 생성 후 깨끗한 폴더에서 `시작하기.pyw --check`
6. 결과물 업로드
   - `SweptPathChecker-windows`(약 36 MB)
   - `SweptPathChecker-source`(약 0.1 MB)
   - 보관 30일

- 최신 성공 실행: https://github.com/danba1229/counting-program/actions/runs/43660332434 (결과물 보관 2026-12-15까지)
- 콘솔 인코딩 문제 때문에 워크플로에 `PYTHONIOENCODING=utf-8`이 설정돼 있습니다.
- 한글 파일 이름은 PowerShell·tar로 다루면 깨집니다. 소스 묶음은 Python 스크립트로 만드세요.

### 배포 시 알려진 문제

- **Chrome 차단**: Chrome이 서명 없는 exe를 "흔하지 않은 파일"로 차단합니다. `chrome://downloads`에서 "계속"을 누르거나 소스 묶음(`시작하기.pyw`)을 쓰면 됩니다.
- **Windows SmartScreen**: 처음 실행할 때 경고가 뜹니다. "추가 정보 → 실행"을 누르면 됩니다. 근본적으로 없애려면 코드 서명 인증서가 필요합니다(유료).

---

## 7. 검증 기록

| 항목 | 결과 |
|---|---|
| 회전반경 재현 | 최대 조향으로 원을 돌면 바깥 앞바퀴 반경 = 최소회전반경 (오차 0.1% 이내) |
| 샘플 도면 90° 코너 | 소형·카운티 가능, 대형·세미트레일러 불가(코너 기둥 충돌) |
| 실제 단지배치도 DXF (55 MB, 선 22만 개) | 기존 ezdxf 방식과 0.1 m 격자 비교 차이 0칸, 레이어 150개 동일, 선 길이 −0.03% (곡선 분할 차이) |
| 도면 열기 (첫 / 캐시) | 55 MB: 4.6초 / 0.25초, 100 MB 시험 도면: 7.8초 / 약 1초, 메모리 0.3 GB 이하 |
| 화면 반응 | 끌기 1회 약 1 ms, 휠 1회 약 4 ms (도면 크기와 무관) |
| 화면 크기 | 1400×860, 1366×768에서 잘리는 버튼 없음 (Linux 가상 화면 기준) |

측정은 클라우드 Linux와 GitHub Windows Server 기준입니다. **사용자 PC(한글 Windows 10/11)에서 직접 확인된 적은 아직 없습니다.**

---

## 8. 알려진 한계

- 차체는 평면 사각형입니다. 차고(높이), 램프 경사, 범퍼 모서리 둥글기는 반영하지 않습니다.
- 세미트레일러 후진은 숙련 운전자보다 보수적으로 "불가"가 나올 수 있습니다.
- 경로 추종은 제어기 기반이라, 경유점을 지나치게 급하게 꺾어 찍으면 차량이 경로를 크게 벗어납니다(주의로 표시됨).
- DWG 직접 열기: GstarCAD 경로는 GstarCAD 2023이 열려 있는 PC에서 쓰도록 만들었습니다. ODA 경로는 실제로 시험한 적이 없습니다.
  GstarCAD가 열려 있는데 새 인스턴스를 띄우면 CO_E_SERVER_EXEC_FAILURE로 실패할 수 있어, 실행 중인 GstarCAD에 먼저 붙습니다.
- 100 MB를 넘는 실제 도면(221 MB, 359 MB 파일이 있음)은 시험하지 못했습니다.
- PDF의 한글 글꼴은 Windows 맑은 고딕을 자동으로 찾습니다. 한글 글꼴이 없는 환경에서는 글자가 깨질 수 있습니다.
- 차량 제원 중 확인이 필요한 값이 있습니다(§9).

---

## 9. 남은 작업

### 사용자 확인·결정이 필요한 것

1. **장애물 레이어 지정**: 실제 단지배치도의 기본값(150개 중 136개 장애물)에는 차로 위 선까지 들어 있어, 지금은 어떤 차량이든 "불가"가 나옵니다. 사용자가 실제 장애물 레이어를 골라 **레이어 설정 파일**로 주기로 했습니다(미수령).
2. **카운티 제원 확인**: 앞·뒤 내민길이(1.2 / 2.425 m), 윤거(1.7 m), 최소회전반경(7.6 m)은 **추정값**입니다(`vehicles.py`).
3. **설계기준자동차 제원**: 최신 개정 기준과 맞는지 확인이 필요합니다. 윤거, 핸들 시간, 세미트레일러 트랙터 뒷부분·트레일러 앞 내민길이는 가정값입니다.
4. **사용자 PC 실행 확인**: 한글 Windows에서 글꼴과 배치 확인, 피드백 반영이 필요합니다.
5. **Drive 정리**: 테스트 때 만든 임시 사본(`임시_테스트용사본_삭제예정.dxf`)이 휴지통에 있습니다. 링크 공유가 켜진 상태이고 다운로드만 막혀 있으니 **영구 삭제가 필요**합니다(사용자에게 요청함).

### 다음 개발 작업 (사용자가 지정한 순서)

**자동 경로 탐색 (다음 작업)**: 출발점과 목적지만 주면 경로를 스스로 찾는 기능입니다. 착수 전에 사용자에게 아래 세 가지를 물었고 **아직 답을 받지 못했습니다**.

1. 입력 방식: 출발·목적지만으로 찾을지, 경유점을 힌트로 줄 수 있게 할지 (권장: 힌트 허용)
2. 후진(여러 번 꺾기)까지 자동으로 시도할지
3. 범위: 단일 차량부터 만들고 세미트레일러는 다음 단계로 할지 (권장)

구현 방향 제안은 다음과 같습니다.
- Hybrid A* 방식으로, 상태는 (x, y, θ)이고 동작은 최대 조향 좌/중/우 × 전진/후진입니다.
- 충돌 검사는 `analysis.build_obstacles`의 STRtree를 다시 씁니다.
- 찾은 경로를 경유점으로 바꿔 기존 `Simulator`로 다시 검증하면, 판정 기준이 하나로 유지됩니다.

### 그 밖의 추천 작업

| 순위 | 작업 | 이유 |
|---|---|---|
| 1 | 브랜치를 main에 합치기(PR), 가능하면 별도 저장소로 분리 | 지금은 작업 브랜치에만 있고 통행량 카운터와 성격이 다름 |
| 2 | GitHub Releases 배포 | Actions 결과물은 30일 뒤 사라지고 로그인이 필요함 |
| 3 | 기존 검토 완료 도면과 결과 대조 | 실제 운전 궤적과의 정확도 검증이 아직 없음 |
| 4 | 코드 서명 | Chrome·SmartScreen 경고 제거 (회사 배포 시) |

---

## 10. 외부 자료 위치

| 자료 | 위치 | 비고 |
|---|---|---|
| 시험용 실제 도면 DXF (55/93/221/359 MB) | 사용자 Google Drive `auto turn` 폴더 | 외부 공유 불가 자료. 받으려면 사용자 승인 필요 |
| 기존 상용 프로그램 폴더 | 같은 Drive 폴더 | **사용하지 않음** (라이선스 문제로 손대지 않기로 함) |
| 클라우드 작업 환경 | 개인 환경 `counting-program` | Drive 다운로드 주소 2개 추가 허용. GitHub 결과물 저장소 주소 허용은 사용자가 거절함 |

---

## 11. 다음 담당자를 위한 체크리스트

- [ ] 이 브랜치를 받아 `python -m pytest -q` (42개)와 `python app.py --selftest`가 통과하는지 확인
- [ ] §9의 사용자 확인 사항 중 답을 받은 것이 있는지 먼저 확인
- [ ] 도면 변환 방식을 바꾸면 `CACHE_VERSION` 올리기
- [ ] 화면을 바꾸면 1366×768에서 버튼이 잘리지 않는지 확인 (아래쪽 요소를 먼저 배치하는 패턴 유지)
- [ ] 푸시 후 Actions의 Windows 빌드와 exe 자체 점검 결과 확인
