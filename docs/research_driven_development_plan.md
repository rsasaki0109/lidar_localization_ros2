# Research-Driven Development Plan

Last updated: 2026-07-13

## 目的

`lidar_localization_ros2` を、公開データで失敗境界を説明でき、安全に復旧できる
ROS 2 向け3D LiDAR地図ローカライザへ育てる。本計画は新しい手法を一度に移植する
ものではなく、既存のNDT/BBS/G3構成を維持しながら、論文から得た知見を比較可能な
実験として導入し、公開データで勝ったものだけをruntimeへ昇格させる。

既存のclaim ruleとdecision gateは
[competitive_roadmap.md](competitive_roadmap.md)を引き続き適用する。本書は2026-07-09
以降の技術優先順位と実験順序を具体化する。

## 現在地と判断

2026-07-13時点で、次の基盤は既にある。

- NDT、NDT_OMP、GICP、small_gicp系backendと公開replay harness
- BBS_2Dによる大域候補生成、候補ごとのNDT scoring、guard付きG3自動再初期化
- IMU preintegrationとdefault-offのcontinuous-time deskew hook
- `/alignment_status`、failure taxonomy、GT評価、false-confirmを防ぐcross-check
- Koide `outdoor_hard_01a` 120 sでGT確認済みの一時的なkidnap recovery

未解決の主因は、機能不足ではなく次の4つの失敗境界である。

1. Koideの旋回区間で再度trackingを失う。Livoxのpoint time spanは約0.2–0.3 sで、
   従来の`scan_period=0.1`由来のguardと不整合だった。2026-07-09にguard ratioは
   configurableになったが、精度改善はまだ実証されていない。
2. 北側の反復道路形状でBBS_2Dが南側corridorのaliasを返す。2D occupancy scoreだけ
   では3D構造の異なる場所を棄却できない。
3. G2候補scoringが遅いと、正しいscan-stamp時刻の候補でもpublish時には古くなる。
4. Boreasは最初の約12 s後にfitness cliffへ入り、crop tuningや閾値緩和では復旧せず、
   map/GT/frame/overlapのどこが原因かまだ分離できていない。

したがって優先順位は、`deskew -> localizability -> 3D global verification -> Boreas`
とする。backend既定値の変更、GPU化、学習ベース手法の導入は先行させない。

## 論文から採用する知見

| 研究 | 得られる知見 | このprojectでの採用範囲 |
| --- | --- | --- |
| [CT-ICP](https://arxiv.org/abs/2109.12979) | scan内の軌跡を連続時間で扱い、運動歪みをregistrationの一部として扱う | まず既存のscan内SE(3)補間を正しく評価し、その後に区分的IMU軌跡を実験variantとして追加する。CT-ICP全体は移植しない |
| [FAST-LIO2](https://arxiv.org/abs/2107.06829) | raw pointとIMUを密結合し、異なるscan patternや高速運動へ対応する | Livoxのpoint timestamp、IMU extrinsics、bias、scan内predictionの整合性を重視する。mapping stackへの置換はしない |
| [On-Manifold IMU Preintegration](https://arxiv.org/abs/1512.02363) | SO(3)上のpreintegration、bias補正、不確かさ伝播 | 現在のGTSAM smootherをdeskew軌跡の情報源として使う際の基準。単一のstart-to-end transformだけで十分かを検証する |
| [LI-Init](https://arxiv.org/abs/2202.11006) | LiDAR運動とIMUを対応付け、time offset、extrinsic、gravity、biasを同時に初期化する | まずofflineでGT姿勢差分とgyro積分のWahba整合、signed-axis permutation、bias、cloud stamp仮説を分離する。runtime calibratorは導入しない |
| [LI-Calib](https://arxiv.org/abs/2007.14759) | asynchronousなLiDAR/IMUをcontinuous-time trajectoryでtargetless 6-DoF calibrationする | offline簡易診断で大きな不整合が出た場合の次段候補。今回のGT依存診断をruntime calibrationとは呼ばない |
| [KISS-ICP](https://arxiv.org/abs/2209.15397) | 単純なmotion compensation、adaptive correspondence threshold、robust kernelでも高い汎用性を得られる | LiDAR-only deskew fallbackとregistration residualのadaptive gateを比較候補にする。default NDTを直ちに置換しない |
| [X-ICP](https://arxiv.org/abs/2211.16335) | correspondenceが各自由度をどれだけ拘束するかを分析し、弱い方向の更新を制限する | fitnessとは独立の`localizability`診断を先に追加し、弱い方向のfreeze/soft constraintは実験variantに留める |
| [Degeneracy field analysis](https://arxiv.org/abs/2408.11809) | passiveな検出だけでなくactive mitigationが必要で、TSVD・制約・正則化に異なる特性がある | 診断のみ、hard freeze、soft regularizationを同じfixtureで比較する。単一eigenvalue閾値を即default化しない |
| [Scan Context++](https://arxiv.org/abs/2109.13494) | 回転と横方向変位に頑健なplace retrievalを、metric registrationの前段に置ける | keyframe/route databaseがある場合の候補領域絞り込みに使う。raw mapだけから無理にdescriptor DBを合成しない |
| [TEASER++](https://arxiv.org/abs/2001.07715) | 大量outlier下でもcertifiableな3D global registrationが可能 | BBS top-K候補のoffline 3D verifier候補。依存・license・実時間性を確認後にsidecarから比較する |
| [Quatro++](https://arxiv.org/abs/2311.00928) | ground segmentationを使い、車両LiDARの疎性・退化下でglobal registrationを安定化する | Koide/Boreasのground vehicle向け3D verification variant。Livoxでgroundが十分観測できるかを先に測る |
| [KISS-Matcher](https://arxiv.org/abs/2409.15615) | Faster-PFH、graph pruning、robust solverを統合しglobal registrationを高速化する | BBS top-Kの3D verifier第一候補。まずartifact生成器として導入し、false acceptとlatencyでTEASER++系と比較する |
| [small_gicp](https://doi.org/10.21105/joss.06948) | preprocessingからregistrationまでの並列化とfine registrationの高速化 | 既存backend comparison結果を維持しつつ、G2 verifier/scorerのlatency候補として使う |
| [Boreas](https://arxiv.org/abs/2203.10168) | 季節・天候差を含むLiDAR、IMU、cm級GTの反復走行 | map splitとcross-season robustnessの主評価データにする |
| [Boreas-RT](https://arxiv.org/abs/2602.16870) | 9 route、60 sequence、643 kmのより厳しいmetric localization評価 | 元Boreasで原因分離と安定化ができた後のgeneralization gate。直近sprintではdownloadしない |

論文の結果はこの実装での性能を保証しない。各知見は必ず既存baselineとのA/B、複数repeat、
公開artifactを通して採否を決める。外部実装を直接利用する場合は、導入前にlicenseと依存関係を
確認する。

## 共通の実験規約

すべてのruntime変更は、先に`experiments/`または独立harnessのvariantとして導入する。

- 同じbag、map、initial pose、reference、replay rate、評価commandを使う。
- stochasticなKoide recoveryは最低3 repeat、default候補は5 repeatで比較する。
- 平均だけでなくmedian、p95、最悪run、成功率を保存する。
- trajectory accuracyだけでなくcoverage、連続reject数、reinitialization回数、false confirm、
  alignment latency、query latency、CPU/loadも記録する。
- GTは評価と原因分析にだけ使い、runtimeのcandidate orderingやacceptanceには使わない。
- 負荷依存のtiming gateは実行前loadが5未満の場合だけclaimに使う。
- 一つの実験で変更する仮説は一つにする。閾値緩和とrecovery変更を同時に混ぜない。
- artifactにはcommit、config、dataset ID、map hash、bag metadata、ROS distroを記録する。

安全上のhard gateは全phase共通である。

- false recovery confirmは0件。
- NaN、crash、unbounded reset loopは0件。
- Istanbul/HDL release regressionを壊さない。
- default-on化は2つ以上の公開datasetで非劣化を確認してから行う。

## Phase R0: Baseline固定と評価窓の切り出し

目的は、長時間replayを毎回回さずに、旋回失敗を再現できるfixtureを作ること。

作業:

1. Koide `outdoor_hard_01a`のbag約90–112 sをcorner windowとして固定する。
2. no-deskew、現行deskew、IMU-preintegration-onlyの3 variantを同一runnerで実行する。
3. point time span、selected field、valid/invalid/clamped count、deskew applied ratio、
   angular speed、alignment resultを1 scan単位でCSVへ保存する。
4. 120 s full runとwindow runで失敗順序が一致することを1回確認する。

完了条件:

- 一つのcommandで3 variantを生成できる。
- baselineが既知のcorner lossを再現する、または再現しない場合はrun varianceとして記録する。
- `scan_time_range_max_duration_ratio=4.0`が単にdeskewを有効化するだけでなく、どのframeに
  適用されたかをartifactから追える。

## Phase R1: 旋回時deskewの改善

仮説: corner lossの主要因は、0.2–0.3 sにまたがるLivox cloudを単一scanとして扱う際の
motion distortion、またはpoint timestampとIMU trajectoryの時間基準不整合である。

比較variant:

| Variant | 内容 | 目的 |
| --- | --- | --- |
| D0 | deskew off | baseline |
| D1 | 現行start-to-end SE(3)補間、ratio 4.0 | 2026-07-09変更の効果確認 |
| D2 | point timeを実測spanで正規化し、reference timeを明示 | timestamp/referenceの誤りを分離 |
| D3 | IMU sample列から区分的poseを作り、point時刻ごとに補間 | CT-ICP/FAST-LIO2から得た連続時間近似 |
| D4 | IMU不成立時のみconstant-velocity LiDAR fallback | KISS-ICP型の安全なfallback検証 |

D2以降はD1が不十分な場合だけ進める。最初から複雑なsplineやfull LIOへ進まない。

主要metrics:

- corner windowのtranslation/rotation error、end error、matched ratio
- 最大連続reject、`reinitialization_requested`の有無、accepted gap
- deskew applied/invalid/clamped ratio
- alignment median/p95とwall time
- full 120 sの`ends_recovered`とfalse confirm

昇格条件:

- corner windowを3/3 repeatで失わず、end translation errorが1 m以下。
- no-deskew比でRMSEまたは成功率が改善し、alignment p95が20%以上悪化しない。
- HDLとIstanbulでaccuracy/coverageのmaterial regressionがない。
- timing fieldが欠落・異常なsensorでは明示的にskipし、誤deskewしない。

### R0/D1 first result (2026-07-13): negative

Koide `outdoor_hard_01a`のbag offset 85 s、duration 27 sを3 repeatし、D0
`lidar_only`とD1 current deskew (`scan_time_range_max_duration_ratio=4.0`)を比較した。
ローカルartifactは`/tmp/lidarloc_koide_corner_deskew_ab_20260713`。

- Gateは**FAIL**。両modeとも成功は1/3で、D1はD0の成功率もaccuracy medianも改善しなかった。
- D0 median: translation RMSE `0.284 m`、end error `1.197 m`。
- D1 median: translation RMSE `0.380 m`、end error `2.026 m`、deskew applied ratio `50%`。
- D1最悪runはRMSE `30.458 m`、end error `40.754 m`。そのrunではdeskew applied ratioが
  `12%`まで落ち、25 scan中21 scanが`imu_preintegration_integration_window_too_large`になった。
- 3 runのscan span p95 medianは`0.200 s`で、最大`0.301 s`を確認した。invalid/clamped pointは0。
- 実行時load averageは約5–6で、latency比は正式なperformance claimに使わない。

解釈: guard ratioを広げてD1を適用可能にするだけでは不十分で、現行の「smootherの最新
start-to-end transformをcloud全体の実測spanへ正規化する」近似はrunによってregistrationを
悪化させる。悪化後はrejectで`last_scan_stamp_for_imu_`が進まず、integration windowが1 sを
超えてdeskew/IMU seedが止まるdeath spiralに入る。したがってD1のdefault-on化は棄却する。

次はD2を単なるreference-time sweepにせず、各point timestampを実際のIMU pose historyへ対応
付ける区分的trajectory prototypeとして実装する。最初にdiagnostic-onlyで、cloudのstart/end
時刻、利用IMU時刻範囲、外挿量を記録し、IMU coverageがcloud spanを満たさないscanはdeskewを
適用しない。

### R1/D2-D4 + localizability proxy result (2026-07-13): all negative

同じKoide `outdoor_hard_01a` offset 85 s、duration 27 sで、次の5 modeを各3 repeatした。
artifactは`/tmp/lidarloc_koide_multimethod_valid_20260713`、集計はその
`summary.json`と`summary.md`にある。

- D0: `lidar_only`
- D1: 現行のsmoother start-to-end補間
- D3: `header stamp + point t`を独立したrotation-only IMU pose historyへ対応付ける区分的補間
- D4: 直前のaccepted LiDAR相対運動を時間換算するconstant-velocity補間
- L0: scan XY共分散の固有値比が0.05未満なら`previous_delta` seedを抑制する軽量proxy

KoideのPointCloud2ではheaderがscan開始、`t`が0から最大0.1--0.4 sのnanosecond offsetで
あることをbagから確認した。D3はscan両端をIMU履歴が完全に覆う場合だけ補正し、coverage
medianは`1.0`、適用率medianは`94.1%`だった。それでも3 runすべて失敗し、translation RMSE
median `18.137 m`、rotation RMSE median `25.145 deg`、最悪end error `40.259 m`、reject
streak worst `10`となった。時刻coverageの修正だけでは補正方向の正しさを保証できない。
rotation-only履歴、IMU extrinsic/bias、scan reference frame、registrationとの結合を次の原因候補
として残し、このvariantはdefault候補にしない。

D4も3/3安定せず、translation RMSE median `0.583 m`、rotation RMSE median `9.552 deg`、
最悪最大誤差`3.553 m`、reject streak worst `8`だった。直前scanの運動を現在scanへそのまま
移すconstant-velocity仮定はこの不均一なLivox message spanでは安全でない。

L0の固有値比は全85 diagnostic rowで最小`0.477`、median `0.745`で、guard発火は`0`だった。
見かけ上のtranslation RMSE medianは`0.129 m`、最悪end errorは`0.052 m`だったが、repeat 1の
coverageは`0.50`、最大誤差は`3.704 m`で、3/3 gateを満たさない。さらにguardが一度も
runtime actionを変えていないため、baselineとの差は実行間ばらつきであり手法の改善claimには
使えない。この軽量なscan形状proxyでは旋回失敗を検出できないため、R2の本命は引き続き
correspondence/Hessian由来の方向別localizabilityとする。

共通採用gateは、3/3 repeat、end translation `<=1 m`、最大translation `<=2 m`、rotation RMSE
`<=5 deg`、trajectory coverage `>=0.8`、reject streak `<=3`、reinitialization 0、baseline比
alignment p95 `<=1.2`とした。全candidateがFAILしたため、既定値はdeskew offのまま維持する。
なお全modeでcrash、NaN、再初期化要求は0だった。共有機のCPU使用率が高いため、latency値は
候補間gateにのみ使い、一般性能claimにはしない。

## Phase R2: Localizability診断と退化方向の抑制

仮説: scalar fitnessだけではcorridor aliasや旋回時の弱拘束方向を判別できない。

順序:

1. registration correspondence/Hessianから6-DoFのeigen spectrum、condition number、
   direction contributionを計算し、diagnostic-onlyで記録する。
2. GT errorの立ち上がりより前に指標が変化するかをKoide/Boreas/HDLで確認する。
3. `diagnostic_only`、`TSVD_freeze`、`soft_regularization`を実験fixtureで比較する。
4. runtime actionは、false positive rateとcross-dataset閾値が決まった後にだけ接続する。

完了条件:

- good/bad alignmentをfitnessとは別軸で分離できる。
- healthy HDL/Istanbulで不要なfreezeを起こさない。
- 弱拘束方向を抑制しても、観測可能な方向の更新を止めない。

### R2 diagnostic result (2026-07-13): negative, mitigation gate closed

NDT_OMPの最終poseを、実際のvoxel correspondence、探索方式、距離guard、priorを保ったまま
`computeDerivatives()`で再評価するdefault-off診断を追加した。NDT scoreは最大化問題なので、
`-sym(H)/correspondence_count`をper-correspondence curvatureとして対称固有分解し、6固有値、
非正固有値数、絶対condition、weak ratio、最弱6-DoF vectorをselected alignment attemptへ記録する。
translationとrotationは単位が異なるため、raw spectrumをcovarianceや物理的情報量とは呼ばない。

Koide `outdoor_hard_01a` offset 85 s、duration 27 sを3回実行したartifactは
`/tmp/lidarloc_koide_hessian_20260713`。38/38 diagnostic rowが有効で非正固有値は0だったが、
causalな退化alarm（4 scan warm-up後、weak ratioが過去medianの半分以下かつconditionが2倍以上）が
GT errorまたはreject streakに先行したrunは`0/3`だった。first reject直前のweak ratioは各runで
`0.024867`、`0.019358`、`0.014098`であり、各runの初期最小値より良い。最弱vectorは多くのscanで
translation x成分が支配的で、旋回失敗をyaw弱拘束として説明できなかった。

診断は最終derivativeをもう一度全点評価するため重い。alignment medianは3 runで
`0.959--1.290 s`、trajectory coverageは`0.415--0.604`だった。既存LiDAR-only 3 runの
`0.351--0.690 s`、coverage `0.556--0.963`に対してonline latency/coverage gateもFAILする。
crash、NaN、reinitialization要求は0だが、この計測方法をdefault-onにはしない。

したがってR2のこの指標は**negative result**とし、目的に定めた条件どおりTSVD freezeとsoft
regularizationは実装・評価しない。X-ICPのdirection contributionはraw NDT Hessian spectrumより
細粒度なので、R2を再開する場合はcorrespondenceごとの方向寄与をoffline/downsampledに集計し、
まずhealthy/failure分離と計測latencyを満たすことを新しい入口条件にする。

## Phase R3: BBS候補の3D検証とG2 latency

仮説: BBS_2Dは高recallなcandidate generatorとして残し、3D geometry verifierを後段に
置けば、北側corridor aliasをfalse publishせずに落とせる。

実験pipeline:

```text
current scan
  -> BBS_2D top-K
  -> candidate-centered 3D map crop
  -> KISS-Matcher / TEASER++ / Quatro++-style verifier
  -> fine NDT or small_gicp score
  -> supervisor safety gates
```

最初の比較はoffline artifactで行う。候補ごとにBBS rank、3D inlier/consistency、refined pose、
fine score、runtimeを保存する。Scan Context++はroute/keyframe databaseが入手できるscenario
でのみretrieval前段として評価し、map-wide claimとroute-crop claimを混同しない。

並行して、既知の`pclomp`問題を独立A/Bする。

- KDTREE + 1 threadをbaselineにする。
- `DIRECT7`を明示設定した1/4 threadを比較する。
- byte parityではなく、candidate rank、fitness、GT seed error、latencyを比較する。
- thread数だけを先に公開parameter化しない。

完了条件:

- Koide north-stretchでtrue candidate recall@Kを維持し、南corridor aliasを全て棄却する。
- verifierを含むquery p95を、移動量がalias capture radiusへ達する前に抑える。
- 3 repeatでfalse confirm 0、bounded attempts、少なくとも2 repeatがend recovered。
- HDL kidnapped-start regressionもpassする。

## Phase R4: Boreas原因分離

Boreasでは閾値sweepを再開する前にdata contractを検証する。

1. mapping/localization sequence、sensor extrinsics、timestamp、frame、GT transform、map hashを
   machine-readable manifestへ固定する。
2. GT poseを使ったoffline oracle cropで、各scanのmap overlapと正解poseでのregistration scoreを
   計算する。これは診断専用でruntimeには入れない。
3. 約12 sのcliffを、`map overlap不足`、`seasonal map差`、`prediction error`、
   `registration basin/degeneracy`、`timestamp/frame error`へ分類する。
4. 原因に応じてR1 deskew、R2 localizability、map split変更のいずれか一つを適用する。
5. 60 s diagnostic -> 120 s confirmation -> 別季節sequenceの順に広げる。

完了条件:

- cliffの直前・直後について、GT poseでのoverlapとscoreを含む原因説明ができる。
- 120 sでmatched coverage 90%以上、translation RMSE 2 m以下を最初の目標とする。
- 目標未達でも、誤ったrecoveryを増やさず、反証された仮説を記録する。

元Boreasで安定した後、Boreas-RTの1 route/2 conditionsを外部妥当性確認として追加する。

## Phase R5: Release gateと既定値判断

候補変更は次のmatrixを満たしたときだけdefault候補にする。

| Dataset | 役割 | 必須結果 |
| --- | --- | --- |
| Koide outdoor hard | 旋回、kidnap、alias、Livox+IMU | corner 3/3、false confirm 0、full recovery boundary明示 |
| HDL sample | 別sensorでのG3/IMU安全性 | kidnapped-start regression pass |
| Autoware Istanbul | no-IMU/Nav2安全性 | release gate pass、deskew変更がinactiveでも非劣化 |
| Boreas | cross-season LiDAR+GT | 原因分離後に120 s gate、別seasonで確認 |

最終成果物:

- one-command regressionとJSON/Markdown summary
- parameter、diagnostic、failure modeの利用者向けdocument
- negative resultを含むexperiment log
- defaultを変える場合はmigration noteとrollback parameter

## 実行順序と目安

| 順序 | Work package | 目安 | 次へ進むgate |
| --- | --- | --- | --- |
| 1 | R0 corner fixture + D0/D1 A/B | 2–3日 | failure再現とartifact completeness |
| 2 | R1 timestamp/reference検証、必要ならD2/D3 | 1–2週 | corner 3/3、no regression |
| 3 | R2 diagnostic-only localizability | 1週 | error precursorとして有効 |
| 4 | R3 offline 3D verifier + latency A/B | 2週 | north alias rejection、false accept 0 |
| 5 | R3 guarded live integration | 1週 | Koide/HDL gate |
| 6 | R4 Boreas cause isolation | 1–2週 | cliff分類と120 s result |
| 7 | R5 cross-dataset regression/release | 1週 | 全hard gate |

目安は1人で既存machineを使うengineering effortであり、dataset downloadやidle-machine待ちは
含まない。各gateで仮説が反証された場合、次phaseへ機械的に進まず本書へnegative resultを追記する。

## 直近の着手項目

最初の実装単位はR0である。

1. 既存`run_koide_hard_imu_deskew_smoke.sh`を再利用し、no-deskew/current-deskewを同じ
   manifestから実行できるようにする。
2. 旋回windowを指定できる引数と、3 repeatのsummaryを追加する。
3. summaryへdeskew適用率、scan time span、reject streak、reinit、trajectory end error、
   latency p95を出す。
4. datasetがない環境でもCLI/unit testを実行でき、実datasetがある環境ではone commandで
   replayへ移れるようにする。
