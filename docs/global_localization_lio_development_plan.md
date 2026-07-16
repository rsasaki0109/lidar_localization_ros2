# Global Localization and LiDAR/IMU Odometry Development Plan

Last updated: 2026-07-16

## 目的

本計画の目的は、`lidar_localization_ros2`を次の一続きの推定系へ発展させることである。

1. LiDARまたはLiDAR + IMUから、連続で局所的に滑らかな`odom -> base_link`を生成する。
2. 既知の3D地図に対する`map -> base_link`観測で長期driftを補正する。
3. 地図追跡を失った場合は、地図全体またはroute databaseから候補を検索する。
4. 3D geometryで候補を検証し、誤復旧を防ぎながら`map -> odom`を再接続する。

NDTを即時廃止する計画ではない。NDT_OMPを既知のbaselineとして残し、非NDT方式を同じ
入出力、同じデータ、同じ評価rubricで比較する。勝者は役割ごとに選ぶ。odometry、局所地図追跡、
大域候補検索、3D検証を一つのbackend名で統一しない。

## 最上位目標: Hard Point Cloud Localization Dataset完走

LiDAR/IMU odometry trackの最上位目標は、Koide Hard Point Cloud Localization Datasetの
outdoor hard系列を、地図補正やglobal reinitializationに頼らず`odom -> base_link`だけで最後まで
走り切ることとする。短い成功区間のRMSE、途中までのtrajectory、GTで並べ替えた結果は完走に
数えない。

### Primary goal

`outdoor_hard_01a`の全380秒を、同じ設定で3 repeat完走する。

**2026-07-16達成:** GLIM CPU GICP + IMU構成は固定設定で3/3 runを通過した。coverageは
各99.210%、start固定translation ATE RMSEは1.529--1.866 m、end errorは3.871--4.603 m、
processing p95は0.0664--0.0884秒（scan period 0.1000秒）だった。再現手順、上流patch、全metricは
[実験記録](../experiments/koide_odometry_glim_gicp6500/README.md)に固定した。

この達成trajectoryは既知地図やGTを使わないが、GLIMが走行中に生成したsubmap/global graphの補正と
ground vehicleのconstant-z制約を含む。したがってproject-level LiDAR/IMU trajectory完走gateの達成とし、
補正前front-endだけの厳密なopen-loop odometryは別gateとして扱う。

必須gate:

| Metric | Target |
| --- | --- |
| trajectory coverage | 各run 99%以上 |
| final timestamp | bag終了の1秒以内までodometryを出力 |
| output continuity | 1秒を超えるpose gap 0、timestamp逆行 0 |
| safety | crash、NaN、TF jump、無断reset 0 |
| accuracy | start固定評価でtranslation ATE RMSE 2 m以下、end error 5 m以下 |
| relative accuracy | 10 m RPE translation median 0.20 m以下、rotation median 1.0 deg以下 |
| realtime | idle machineでprocessing p95がscan period以内、queueの非有界増加なし |
| repeatability | 3/3 runが上記必須gateを通過 |

ATE/RPE thresholdは最初の正しいbaseline artifactを作った後も、結果に合わせて緩めない。sensor/GTの
時間同期誤差が評価floorを支配すると証明された場合だけ、根拠と旧・新両方の値を残して改訂する。

### 完走evidenceの作り方

各runでは推定trajectory、GT reference、runner summary、runtime計測を保存し、次で単発gateを判定する。

```bash
python3 scripts/check_koide_odometry_completion.py \
  --estimated-csv results/run_01/pose_trace.csv \
  --reference-csv data/public/koide_hard_localization/outdoor_hard_01a/reference.csv \
  --runtime-json results/run_01/runtime.json \
  --run-summary-json results/run_01/summary.json \
  --output-json results/run_01/odometry_completion.json
```

`runtime.json`は少なくとも次の型を持つ。欠損値を成功扱いにはしない。

```json
{
  "processing_p95_sec": 0.08,
  "scan_period_sec": 0.10,
  "queue_growth_unbounded": false,
  "tf_jump_count": 0,
  "unauthorized_reset_count": 0
}
```

`summary.json`は既存benchmark runnerの`target_process_died_during_run`、`return_codes.bag_play`、
`bag_stopped_by_runner`を使う。trajectory CSVは`stamp_sec`、`position_x/y/z`、
`orientation_x/y/z/w`列を持つ。GTは評価時にだけ読み、odometry入力や初期姿勢補正には使わない。

3つの独立runを同じ閾値で通したことは次で確定する。集約器は全run成功、同一threshold、異なる
推定CSVを必須とする。

```bash
python3 scripts/summarize_koide_odometry_completion.py \
  --completion-json results/run_01/odometry_completion.json \
  --completion-json results/run_02/odometry_completion.json \
  --completion-json results/run_03/odometry_completion.json \
  --output-json results/outdoor_hard_01a_3repeat.json
```

### Dataset completion goal

Primary goal通過後、次の4 sequenceを同一backend family、同一default parameter方針で完走する。

- `outdoor_hard_01a`
- `outdoor_hard_01b`
- `outdoor_hard_02a`
- `outdoor_hard_02b`

**2026-07-16経過:** GLIM CPU GICP6500の同一固定設定で01b/02a/02bを各3+1 run実行し、全12 runが
coverage 99%以上、最終到達、crash/NaN/reset/pose jump 0、distance-normalized drift 0.48--0.80%で
完走した（1%目標以内）。requested durationは01aと同じ「GT reference spanの整数切り捨て」規則で
302/363/298秒に固定した。01a primary-goal用の固定閾値に対しては、01b/02bの10 m RPE translation
median（0.22--0.27 m対0.20 m）と02aのtranslation ATE RMSE（2.19--2.44 m対2.0 m）が一貫して超過して
おり、これは閾値を緩めず個別sequenceの観測値としてそのまま記録する。realtime gateは計測中に別の
高負荷プロセス（load average約9）が混入したため未確定であり、idle machineでの3 repeat再計測待ち。
なお保存版runner scriptに`auto_quit`パラメータが漏れており、bag再生完了後にglim_rosbagが永久spin
する再現バグを発見・修正した（01a検証時のrunは別コマンドラインで実行されていた）。

**2026-07-17完了:** quiet machine（1-min load < 2.5、外部常駐~1コア1本まで、各runに`load.log`証跡）で
9 runを再計測し、realtime gateを含む全plan-levelゲートに9/9合格した。p95は0.034--0.044 s（予算0.1 s）
でrealtime gate確定。drift 0.46--0.68%。固定厳格閾値に対する超過は従来どおり01b/02bのRPE_t
（0.211--0.276 m）と02aのATE（1.884--2.341 m、run1は全厳格ゲート合格＝閾値近傍の確率的挙動）。
open-loop（global correction非適用）exportでは01b/02bのRPE_tが0.165--0.174 mに改善して0.20 mゲートを
通過し、02bはend errorも改善する一方、01a/02aのend errorはcorrectionありが優る。すなわちper-submap
correctionのsubmap境界不連続が10 m RPEを悪化させており、「odometryはodom frameで連続、補正は
map->odomで下流適用」という本計画のアーキテクチャ原則を実データで裏付けた。02aのATE超過は特定の
コーナーでの破綻ではなく、経路全体に一様な水平drift蓄積（t≈170--190 sで2 m超、最終直線で
4.1--4.8 mピーク、3 run同一地点で再現）であり、front-end単体チューニングよりmap照合補正で吸収する
のが正道。詳細はexperiments/koide_odometry_glim_gicp6500/README.mdの2026-07-17節を参照。

各sequenceでcoverage 99%以上、最終到達、crash/NaN/reset 0を必須とする。accuracy gateは各route長と
GT coverageをmanifestに固定し、distance-normalized translation drift 1%以下を共通目標にする。
個別sequenceだけに効くparameter setをdefault候補にしない。

`outdoor_kidnap_a/b`はodometry完走gateに含めない。物理的kidnap/pose discontinuityは局所odometry
だけでは観測できないため、G5のglobal recovery gateとして評価する。odometry側の責務はkidnap時に
偽の高信頼を出さず、healthをdegraded/lostへ遷移させ、復旧後も`odom`座標系を連続に保つことである。

## 現在地

現在のruntimeは地図localizerが中心である。

- `NDT`、`NDT_OMP`、`GICP`、`GICP_OMP`、`SMALL_GICP`、`SMALL_VGICP`を選択できる。
- IMU preintegration、twist、外部`/odom`はregistration seedを改善する予測源として使われる。
- Nav2 modeでは外部`odom -> base_link`を前提に、localizerが`map -> odom`を計算する。
- BBS_2D候補生成、候補ごとの3D score、guard付きG3再初期化がある。
- Koide/HDLで一時的な復旧は確認済みだが、north-corridor alias、query latency、長時間追跡が
  未解決である。

不足しているのは、地図localizerから独立して連続poseを出すodometry front-endと、front-endを
交換してもG2/G3を評価できる明示的な契約である。

## 採用するアーキテクチャ

```text
PointCloud2 + IMU
        |
        v
odometry front-end ---------------------> odom -> base_link
  KISS-ICP / LIO candidate / wheel odom          |
        |                                        |
        +--> deskewed scan + covariance + health|
                                                 v
3D map tracker ---------------------------> map -> odom
  NDT_OMP / SMALL_GICP / candidate backend       |
        |                                        |
        +--> tracking health                     |
                  |                              |
                  v                              |
global candidate retrieval                      |
  BBS_2D / route descriptor                      |
                  |                              |
                  v                              |
3D candidate verification                       |
  bounded NDT / SMALL_GICP / robust verifier    |
                  |                              |
                  v                              |
guarded recovery supervisor --------------------+
```

設計原則:

- odometryは`odom`座標系で連続に保つ。global recovery時にodometry stateやTFをjumpさせない。
- global correctionは`map -> odom`だけを更新する。
- localizerはfront-endの内部状態へ依存せず、timestamp付きpose、twist、covariance、healthを受け取る。
- raw scanとdeskew済みscanを別topicにし、同じscanへ二重deskewを適用しない。
- GTは評価にだけ使い、candidate生成、ordering、runtime acceptanceには使わない。
- 新方式は最初にsidecarまたは`experiments/`で比較し、勝者だけをruntime default候補にする。

## Runtime契約

### Odometry出力

最初にbackend共通adapterを作り、次を固定する。

| Interface | Contract |
| --- | --- |
| `/local_odometry/odom` | `nav_msgs/msg/Odometry`、parent=`odom`、child=`base_link` |
| `odom -> base_link` | odometry nodeだけがpublishする。単一publisherを強制する |
| `/local_odometry/cloud` | 任意のdeskew済みcloud。元scan stampとframeを保持する |
| `/local_odometry/status` | initialized、tracking、degraded、lost、IMU coverage、latency、drop count |
| reset service | map recoveryでは呼ばない。sensor restartまたは明示的operator操作だけに限定する |

covarianceが信頼できないbackendはゼロ行列を「高信頼」の意味でpublishしてはならない。
unknownを明示するか、評価済みerror floorを設定する。

### Map tracker入力と出力

- seedはscan stampへ補間した`T_odom_base(t_scan)`から作る。
- scan受信時点ではなく、scan stamp基準で`T_map_odom * T_odom_base`を初期値にする。
- accepted map poseから`T_map_odom = T_map_base * inverse(T_odom_base)`を更新する。
- reject中も`odom -> base_link`は進めるが、`map -> odom`は最後のaccepted値を保持する。
- odometry age、covariance、healthがgate外ならseed sourceを明示的にfallbackする。

### Recovery state machine

```text
TRACKING -> DEGRADED -> LOST -> RETRIEVING -> VERIFYING -> PROBATION -> TRACKING
                    \-> COOLDOWN <-------------------------------/
```

`PROBATION`ではglobal候補を即確定しない。一定時間のmap score、odometryとの相対運動整合、
pose jump、再rejectの有無を確認し、失敗時は前の`map -> odom`へ戻す。attempt数、cooldown、
候補数は必ずboundedにする。

## 候補方式と優先順位

### A. Local odometry

| Candidate | 役割 | 長所 | 制約 | 判断 |
| --- | --- | --- | --- | --- |
| 現行IMU preintegration + accepted LiDAR pose | control | 既存実装と診断を再利用できる | 独立odometryではなく、map registration rejectの影響を受ける | baselineとして維持 |
| KISS-ICP | LiDAR-only odometry baseline | upstreamがROS 2を公式サポートし、MIT license。IMU不調時の分離試験に向く | IMU biasや高dynamic motionを直接扱わない | **最初にsidecar統合** |
| FAST-LIO2 | tightly-coupled LIO candidate | raw point + IMUのiESKF、Livox系を含む複数scan patternを対象とする | upstreamはROS 1中心、GPL-2.0。本BSD packageへlink/vendorしない | **外部process比較の第一LIO候補** |
| LIO-SAM ROS 2 branch | factor-graph LIO comparator | LiDAR odometry、IMU bias、GPS/loop factorを分離でき、BSD-3-Clause | GTSAM依存、point timeと高品質9-axis IMU前提が強い | FAST-LIO2との比較候補 |
| Point-LIO | high-bandwidth stress candidate | point単位更新、高dynamic/vibrationを主対象とし、BSD-3-Clause系license | upstreamはROS 1/Noetic中心でpoint timestamp依存が強い | Phase O4以降へ延期 |

一次資料: [KISS-ICP upstream](https://github.com/PRBonn/kiss-icp)、
[FAST-LIO2 paper](https://arxiv.org/abs/2107.06829)、
[FAST_LIO upstream](https://github.com/hku-mars/FAST_LIO)、
[LIO-SAM paper](https://arxiv.org/abs/2007.00258)、
[LIO-SAM upstream](https://github.com/TixiaoShan/LIO-SAM)、
[Point-LIO upstream](https://github.com/hku-mars/Point-LIO)。

FAST_LIOのGPL-2.0 codeはrepoへcopy、static link、vendorしない。評価時は別workspace/processで
起動し、標準ROS messageだけで接続する。Point-LIOとLIO-SAMはBSD-3-Clause系だが、ROS 2対応、
transitive dependency、配布形態を含むlicense reviewはruntime dependency化の前に改めて行う。

### B. Local map tracking

| Candidate | 現状 | 次の評価 |
| --- | --- | --- |
| NDT_OMP | 長時間Koideで現時点の推奨baseline | KDTREE/DIRECT7、1/4 threadを分離A/B |
| SMALL_GICP | 既にoptional backendとして統合済み。easy windowでは高精度、full runではlost | odometry seedを共通化し、scan downsampleとmap cropをNDTと同条件にして再評価 |
| SMALL_VGICP | full-window robustness gateで不採用 | odometry層完成までは追加sweepを行わない |
| GICP_OMP | 選択可能だが主比較の証拠が弱い | SMALL_GICPの対照としてsmokeだけ維持 |

small_gicpはICP、plane ICP、GICP、VGICPを提供しMIT licenseであるため、NDT以外の地図追跡を
試す最初の実装候補に適する。ただし既存Koide full runの失敗を無視してdefault化しない。
一次資料: [small_gicp upstream/JOSS implementation](https://github.com/koide3/small_gicp)。

### C. Global candidate retrieval

| Candidate | 適用範囲 | 判断 |
| --- | --- | --- |
| BBS_2D | raw map + current scanだけでmap-wide探索 | map-wide fallbackとして維持 |
| route/keyframe descriptor | mapping時のkeyframe databaseがあるroute localization | BBS前段の第一候補。map-wide claimと分ける |
| Scan Context/Scan Context++ family | yawに頑健なplace retrieval | algorithm比較には入れるが、公開codeはCC BY-NC-SAのためcopyしない |
| learned descriptor | 十分なdomain dataとGPUがある場合 | classical baselineを超える証拠が出るまで延期 |

Scan Contextはplace retrieval用descriptorでありmetric pose solverではない。候補検索後に必ず3D検証を
置く。一次資料: [Scan Context paper/code page](https://github.com/gisbi-kim/scancontext)、
[Scan Context++ paper](https://gisbi-kim.github.io/publications/gkim-2021-tro.pdf)。

### D. Global 3D verification

| Candidate | 役割 | 現在の判断 |
| --- | --- | --- |
| bounded NDT score | 現行baseline | 既存Koide比較でKISS-Matcherより良かったため維持 |
| bounded SMALL_GICP | 非NDT fine verifier | crop、seed、point budgetを同一にして最初に比較 |
| KISS-Matcher | feature + pruning + robust global pose | 既存実験ではsafe abstainだが遅く、採用gate不通過。条件を変えず再sweepしない |
| TEASER++ | extreme outlier下のrobust solver | correspondence生成と合わせたoffline variantに限定 |
| Quatro++ | ground vehicle向けcoarse alignment | ground観測率が十分なspinning LiDAR scenarioだけで比較 |

TEASER++は入力correspondenceをrobustに解くsolverであり、retrievalやcorrespondence生成の代替ではない。
Quatro++はground segmentationを利用するため、Livoxの不均一scanで同じ利点を仮定しない。
一次資料: [TEASER++ paper](https://arxiv.org/abs/2001.07715)、
[TEASER++ upstream](https://github.com/MIT-SPARK/TEASER-plusplus)、
[KISS-Matcher paper](https://arxiv.org/abs/2409.15615)、
[KISS-Matcher upstream](https://github.com/MIT-SPARK/KISS-Matcher)、
[Quatro++ paper](https://arxiv.org/abs/2311.00928)。

## 評価指標

### Odometry

- trajectory coverage、timestamp monotonicity、TF discontinuity count
- ATEに加えて1 m / 10 m / 100 m区間のtranslation・rotation RPE
- distance-normalized drift、velocity RMSE、停止時drift
- IMU initialization time、bias stability、IMU coverage、deskew coverage
- pose latency median/p95/max、scan deadline miss、CPU、RSS、queue drop
- IMU dropout、point-time欠落、sensor restart時のfail-closed挙動

### Map tracking

- accepted/total、pose rows、longest lost window、最大連続reject
- translation/rotation RMSE、end error、false healthy duration
- seed source別の成功率、seed age、alignment latency p50/p95
- `map -> odom` jump、correction rate、odometryとのinnovation

### Global localization and recovery

- labeled queryに対するrecall@K、candidate precision、alias confusion matrix
- query latency p50/p95、candidate age、CPU/RSS
- false publish、false confirm、attempt count、reset loop
- time-to-recover、probation pass率、recovery後30/60秒の追跡継続率

RMSEはcoverageとlost windowの後に読む。短い正解区間だけを出力する方式を勝者にしない。

## Dataset matrix

| Dataset | 主目的 |
| --- | --- |
| Koide outdoor hard 01a | **LiDAR/IMU odometry primary goal: 全380秒を3/3完走** |
| Koide outdoor hard 01b/02a/02b | dataset completion、別route/走行条件への一般化 |
| Koide outdoor kidnap a/b | odometry完走から分離し、実kidnap、corridor alias、G3を評価 |
| HDL sample | 別LiDARでのodometry/IMU安全性、kidnapped-start、throughput |
| Autoware Istanbul | no-IMU fallback、Nav2/TF、地図追跡の非劣化 |
| Boreas | LiDAR + IMU + GT、cross-season、長区間odometryとmap overlap |

一つのdatasetだけでdefaultを変更しない。Boreasはmap/frame/GT contractと12秒cliffの原因が確定するまで
backend rankingに使わない。

## 開発フェーズ

### O0: Contractとfixture固定 — 1週

作業:

1. odometry共通topic、TF ownership、status schema、reset semanticsを定義する。
2. scan/IMU/GTを一回読み、複数backendへ同じtimestamp順で渡すrunnerを作る。
3. Koide 30秒、85--112秒corner、kidnap、HDL、Istanbulのmanifestを固定する。
4. ATE/RPE、coverage、latency、resourceを同じsummary schemaへ出す。
5. `outdoor_hard_01a`全380秒のodometry-only primary-goal manifestとgate checkerを作る。

Exit gate:

- backendを変えても入力hash、scan数、評価windowが一致する。
- TF authorityの重複をtestで検出できる。
- dataset無し環境でcontract/CLI testが動く。

### O1: LiDAR-only odometry baseline — 1--2週

作業:

1. KISS-ICP ROS 2を別package/sidecarとして起動するadapterを作る。
2. `/local_odometry/odom`、TF、health、任意deskew cloudへ正規化する。
3. IMU offでKoide/HDL/Istanbulを実行し、現在のprevious-delta/twist seedと比較する。
4. odometryがlostしてもmap trackerが無条件に追従しないhealth gateを追加する。

Exit gate:

- coverage 99%以上、TF backward jump 0、NaN/crash 0。
- p95 processing timeがscan period内に収まる。
- 2 datasetで現在のLiDAR relative-motion seedよりRPEまたはreject streakが改善する。

### O2: LiDAR/IMU odometry bake-off — 2--3週

状態: **primary full-sequence gate完了**。GLIM CPU GICP + IMUのself-map graph trajectoryで3/3通過。
open-loop comparator、IMU fault injection、2 dataset一般化は引き続き未完了であり、runtime default変更の
根拠にはまだ使わない。

比較条件:

- current guarded preintegration
- KISS-ICP LiDAR-only
- FAST-LIO2 external process
- LIO-SAM ROS 2 branch

作業:

1. IMU axis、unit、extrinsic、gravity、point timestamp、cloud stamp referenceをmanifestへ固定する。
2. 各backendのodometryを共通adapterへ接続し、map correction無しのopen-loopで比較する。
3. IMU dropout、0.5/1.0秒gap、初期静止不足、timestamp offsetをfault injectionする。
4. 勝者をmap trackerのseedへ接続し、closed-loopで再評価する。
5. short windowで勝った上位2方式だけを`outdoor_hard_01a`全380秒へ進め、3 repeatする。

Exit gate:

- 最上位目標の`outdoor_hard_01a` 380秒gateを3/3で通過する。
- LiDAR-only比で2つ以上のLiDAR + IMU + GT windowにおいてRPE medianを20%以上改善する。
- worst-run coverageを悪化させず、IMU dropout時にunbounded poseを出さない。
- p95 latencyがscan deadline内、odometry output gapがNav2 control周期の2倍以内。
- license、ROS 2 packaging、Jetson buildの方針が決まる。

未達ならKISS-ICPまたは外部wheel odomを正式odometry sourceとし、内部IMUはguard付きseed補助に留める。

### O3: 非NDT map tracking — 1--2週

作業:

1. O2で固定した同じodometry seedをNDT_OMP、SMALL_GICP、GICP_OMPへ渡す。
2. scan downsample、map crop、iteration budget、acceptanceを可能な範囲で揃える。
3. easy、corner、long full runを分け、同じbackendを全windowで評価する。
4. backend固有scoreを共通の`RegistrationResult`へ正規化し、score値そのものをbackend間で比較しない。

Exit gate:

- 2公開datasetでcatastrophic lost windowがない。
- baseline比でcoverage非劣化、RMSE 10%以上悪化なし。
- robustnessまたはp95 latencyの少なくとも一方で明確な改善がある。

### G4: Retrievalと3D verifier再設計 — 2--3週

作業:

1. BBS_2D top-K recallを固定し、query latencyのKDTREE/DIRECT7 A/Bを完了する。
2. route databaseがある場合だけdescriptor retrievalを追加する。
3. bounded NDTとbounded SMALL_GICPを同じcrop/seed/point budgetで比較する。
4. north/south corridor confusion setを作り、height histogram、ground/vertical structure、multi-scan
   submapの順に一仮説ずつ追加する。
5. TEASER++/KISS-Matcher/Quatro++はoffline pluginとしてのみ比較する。

Exit gate:

- true candidate recall@K 95%以上。
- 既知alias queryでfalse accepted candidate 0。
- query p95がalias captureまでの時間budget内。
- GTを使わず同じcandidate orderingを再現できる。

### G5: Odometry-aware guarded recovery — 2週

作業:

1. global candidateのscan stampからpublish時刻までodometryでforward propagateする。
2. candidate poseから`map -> odom`を計算し、odometry stateをresetしない。
3. probation中にmap score、relative-motion consistency、pose jump、covarianceを確認する。
4. failed candidate時にatomic rollbackし、bounded cooldownへ入る。

Exit gate:

- Koide real kidnap 3 repeatでfalse confirm 0、少なくとも2 repeatが60秒追跡継続。
- HDL kidnapped-start pass、Istanbul no-IMU path非劣化。
- reset loop、TF dual publisher、backward timestamp、NaNが0。

### O4/G6: Hardwareとrelease gate — 1--2週 + hardware待ち

1. MID-360 + Jetsonでcold start、歩行振動、旋回、IMU saturation、thermal throttlingを計測する。
2. Koide、HDL、Istanbul、Boreasのrelease matrixをidle machineで実行する。
3. 勝者、fallback、rollback parameter、既知failure boundaryをdocument化する。
4. 2公開datasetとhardware gateを通った機能だけdefault候補にする。

全体目安は1人で10--14週。dataset download、Boreas mount、Jetson/MID-360 hardware待ちは含まない。

## 最初の2週間の具体的backlog

1. `LocalOdometryStatus`のrclpy非依存schemaとstate transition testを作る。
2. odometry adapterのtopic/TF contract testを作る。
3. KISS-ICP sidecar用launchとremapだけを追加し、upstream codeはvendorしない。
4. `benchmark_odometry_from_manifest`を追加し、ATE/RPE/coverage/最終timestamp/最大gap/latency summaryを出す。
5. Koide先頭30秒と85--112秒でKISS-ICP vs current seedを各3 repeatする。
6. FAST-LIO2/LIO-SAM用manifest schemaを作るが、結果が出るまでruntime dependencyへ追加しない。
7. global query artifactへ`odom_pose_at_scan`、`odom_pose_at_publish`、`candidate_age_sec`を追加する。
8. `check_koide_odometry_completion`を追加し、380秒完走gateをmachine-readable JSONで判定する。
9. `summarize_koide_odometry_completion`で独立3 run、同一threshold、異なるartifactを検証する。

## Default変更のdecision gate

default変更には次をすべて要求する。

- crash、NaN、TF conflict、unbounded recovery loopが0。
- false recovery confirmが0。
- 2つ以上の公開datasetでcoverage非劣化。
- worst runとlongest lost windowがbaselineより悪化しない。
- accuracy、robustness、latencyのいずれかが実用上明確に改善する。
- Humble/Jazzy build、optional dependency無しbuild、rollback設定が通る。
- 利用licenseと配布方法がBSD-2.0 packageの方針と両立する。

条件未達の方式は失敗ではなく、用途を限定したexperimental backendまたはnegative resultとして残す。
