# 引き継ぎ資料: GLIM 外部オドメトリ・フロントエンド構成 (→ Codex)

作成: 2026-07-18。作業ワークスペース: `/home/sasaki/workspace/old_~2026/lidarloc_ws`(リポジトリは `src/lidar_localization_ros2`、ブランチ `main`、最新コミット `be1491c`)。マスタープランは `docs/global_localization_lio_development_plan.md`。

## ⚠️ 着手前に必ず確認

1. **Claude のバックグラウンドエージェントが作業ツリーを編集中の可能性がある**(2026-07-18 07:03 時点で `src/lidar_localization_component.cpp` の `selectRegistrationSeed` を実装中)。着手前に `git status` / `git diff` で状態を確認し、必要ならユーザーにエージェント停止を依頼すること。中途半端な編集が残っている場合は下の「未完タスク A」仕様と突き合わせて判断する。
2. **未コミット変更が9ファイル+新規1ディレクトリある**(内容は「未コミット変更の内訳」参照)。これらは意図した作業成果であり、破棄しないこと。

## ゴール

Koide Hard Point Cloud Localization Dataset の outdoor_hard 4シーケンス(01a/01b/02a/02b、実長 380/302/363/298 秒)で、フル区間のロバストなローカライゼーションを達成する。単独ローカライザ(NDT_OMP)はどのシーケンスも途中の縮退区間でロックを失い回復できないことが実証済み(SMALL_GICP はさらに悪い)。そこで:

- **GLIM**(CPU GICP 版 LIO、docker)を外部フロントエンドとして `odom→livox_frame` TF を出す
- **ローカライザ**は `map→odom` のみ推定・配信(`enable_map_odom_tf`)
- **G2**(BBS大域自己位置推定)+ **G3**(再初期化スーパーバイザ)がリカバリを担当
- ロック喪失中は「凍結した map→odom × GLIM の生きた odom→base_link」の合成(= odom ブリッジ)で姿勢を継続

## 確定した知見(再発見に時間を溶かさないこと)

1. **FastDDS SHM は docker⇔ホスト間で全データを双方向に無音ドロップする**(`--network=host --ipc=host` でも!discovery は通るので `ros2 topic info -v` は正常に見える)。**全プロセス(ホスト側 bag play / launch / recorder と docker run の `-e`)に `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` を設定**。これ以前の「GLIMフロントエンド」ランは全て GLIM が点群を受信すらしていなかった。
2. **glim_rosnode は `glim_world→odom` TF を無条件配信する**(rviz_viewer.cpp、無効化フラグなし)→ `odom` の親が二重になり TF が壊れる。対策: コンテナ内で `/tf`→`/glim/tf_raw` にリマップし、ホスト側リレー `glim_tf_relay.py` で `odom→livox_frame` エッジのみ `/tf` に転送(実装済み・検証済み)。
3. **`use_imu_preintegration` は必ず true**。launch 引数が yaml を黙って上書きするので両方に設定(ラッパーで対応済み)。false だと縮退区間前でも劣化する。
4. ラッパーの初期ポーズは `INITIAL_POSE_YAML` 環境変数で渡す(以前は 01a 固定というバグがあった)。
5. マシン習慣: ROS_DOMAIN_ID は **101 以下**(それ以上は ephemeral port 衝突で壊れる)。ベンチマークは 1分ロードアベレージ **< 2.5** のアイドルゲート必須(ユーザーの GNSS PPP 等の重いジョブと共存)。sandbox 内では DDS discovery が壊れる(Claude 固有事情だが、環境変数の効かない環境に注意)。Koide の Livox は `imu_accel_scale=9.80665` が必要。
6. GLIM ライブノードは正常時 ~19-35ms/フレームで処理(リアルタイム余裕あり)。GLIM 単体のオドメトリドリフトは 0.5-0.8%。

## 現在の到達点(数値)

- 60秒スモーク(01a 序盤): **ok率 98.2%、RMSE 0.207 m**(全部品動作: GLIM TF 受信、map→odom 配信、ブリッジポーズ配信)
- 01a フル 380 秒: **t=0〜100 秒は ok率 100%**(旧構成は76秒で死亡)→ t≈102 秒の縮退区間でロック喪失 → odom ブリッジ再シードは発火するが **unconfirmed**(正しいシードでも NDT がその区間でマッチ不能)→ 偽の recovery_confirmed → G3 スタンドダウンで以後回復せず。最終 ok率 30.5%、RMSE 18.3 m。
- 4本の共通パターン: 序盤は良好 → 縮退区間で喪失 → 一発再シードでは回復不能(01b 10.0% / 02a 46.6%(後半部分回復)/ 02b 15.4%)。
- ラン成果物: `/tmp/claude-1000/-home-sasaki-workspace-old--2026-lidarloc-ws/cfc30362-*/scratchpad/fullseq/`(**/tmp なので再起動で消える**。必要な数値は本資料と各 `health_summary.md` から回収を)。

## 未コミット変更の内訳(git status 9ファイル+新規ディレクトリ)

- `src/lidar_localization_component.cpp` / `include/.../lidar_localization_component.hpp`:
  - `publishMapToOdomTransform` に freeze_as_last_good フラグ、受理マッチ時のみ `last_good_map_to_odom_` を凍結キャッシュ
  - `republishFrozenMapToOdomTransform()`: 拒否中も毎スキャン凍結 map→odom を再スタンプ配信(AMCL方式)
  - `publishOdomBridgePose()`: 合成ポーズを `/odom_bridge_pose`(KeepLast(1)/transient_local/reliable)で配信。スーパーバイザが TF を直接 lookup しない設計にした理由: 第3プロセスへの /tf 到達がこの環境で不安定なため(上記知見1と同根)
  - (エージェントが実装中)`selectRegistrationSeed` へのブリッジシード統合 — 未完タスクA
- `include/.../registration_seed_policy.hpp` + `test/test_registration_seed_policy.cpp`: シード選択ポリシーの拡張(実装中)
- `scripts/reinitialization_supervisor_policy.py` / `_node.py`: odom_bridge 候補源(opt-in `use_odom_bridge_candidate`、BBS より先に試行、専用試行予算 `odom_bridge_max_attempts`、鮮度 `odom_bridge_max_age_sec`、失敗で BBS フォールバック)。ポリシーテスト 56/56 合格
- `launch/lidar_localization.launch.py`: `odom_topic` 引数 + `enable_map_odom_tf`/`use_odom` の実引数化
- `launch/global_localization_recovery.launch.py`: 上記 + supervisor_odom_bridge_* 引数群のパススルー
- `test/test_reinitialization_supervisor_policy.py`: ブリッジ分岐のテスト追加
- `param/odometry/glim_koide_outdoor_gicp6500_livetf/`(新規): ライブ GLIM 用設定(config_ros.json の map_frame_id を "glim_world" に変更した variant)

既存テストの古い前提（cross-check既定値、wall seed fallback、trusted pose履歴）を現行契約へ
合わせた。`test_reinitialization_supervisor_policy.py` + node ROS integration + harness は現在
76/76 合格しており、以前記録されていた7件失敗は解消済み。

## 実行ハーネス(このリポジトリ内にコピー済み)

`experiments/koide_glim_frontend_localization/` に一式(元は /tmp の scratchpad、パスは修正済み):

- `glim_frontend_system.sh` — システム起動ラッパー。GLIM コンテナ(`docker run -d --name glim_frontend_$$` + trap で `docker rm -f`)、TF リレー、`ros2 launch global_localization_recovery.launch.py`(全フラグ設定済み)。ヘッダコメントに経緯のポストモーテムあり。env: `OUT_DIR`(必須)/ `ROS_DOMAIN_ID`(必須)/ `INITIAL_POSE_YAML` / `GLIM_IMAGE`
- `glim_tf_relay.py` — `/glim/tf_raw` → `/tf` の odom→livox_frame エッジ限定リレー
- `fullseq_glim_frontend_{01a,01b,02a,02b}.yaml` — `scripts/benchmark_from_manifest` 用マニフェスト(`system.command` 方式、出力 `/tmp/glimfrontend_runs/`、ドメイン 71/76/77/78)
- `fullseq_glim_frontend_01a_udp_smoke.yaml` — 60秒スモーク(ドメイン82)
- `idle_glimfrontend_suite.sh` — アイドルゲート付き 4本一括実行

実行例:
```bash
source /opt/ros/jazzy/setup.bash && source ~/workspace/old_~2026/lidarloc_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd ~/workspace/old_~2026/lidarloc_ws/src/lidar_localization_ros2
python3 scripts/benchmark_from_manifest --manifest experiments/koide_glim_frontend_localization/fullseq_glim_frontend_01a_udp_smoke.yaml
```
GLIM イメージ: `lidarloc/glim-ros2:jazzy-v1.2.2-instrumented-guarded`(coreset 版 `-coreset` も有り。データセット: `/media/sasaki/aiueo/datasets/koide_hard_localization/`)

## 未完タスク(優先順)

**A. 外部 odom を運動モデルにする(最重要、エージェントが実装途中)**
喪失中は毎スキャン、合成ブリッジポーズ(凍結 map→odom × 現在 odom→base_link — `publishOdomBridgePose` と同じ計算)を NDT の初期推定に使い続ける。正常追跡中も odom の「前回受理スキャンからのデルタ」を予測に使う opt-in(`use_odom_tf_prediction` 等、デフォルト false)。効果: 縮退区間でもシードが真値に貼り付き(GLIM ドリフト 0.5-0.8%)、環境回復と同時に自動再アタッチ。G3 ブリッジ候補はバックストップとして残す。

**B. 喪失中のポーズ出力継続**
ベンチマークのカバレッジ/RMSE は `/pcl_pose` 由来。喪失中の正しい出力はブリッジポーズ(TF の map→base は凍結再配信で既に継続)。opt-in(`publish_bridge_pose_when_lost` 等)で `/pcl_pose` にもブリッジポーズを出す。内部状態を汚染しないよう注意。

**C. 偽 recovery_confirmed とその後の沈黙の修正**
01a で誤ポーズのまま confirm → スタンドダウン → C++ 側 `reinitialization_requested` が250秒間再アサートされず。要求条件の再発火(confirm 後も failure-like が続けば再要求)か、confirm 基準の強化(クロスチェック fitness + N 回安定 ok の AND)。ポリシーのユニットテスト追加。

**合格基準**: 各シーケンスで matched カバレッジ 90%台後半、RMSE ~0.2-0.5 m(縮退区間はブリッジで橋渡し)、リカバリが偽 confirm なしで成立。検証は 60秒スモーク → 01a フル 380秒 → 3本横展開の順。**アイドルゲート(load<2.5)と UDPv4 を忘れずに。**

**バックログ**(このトラックの後): 01b 急回転(~40°/s)での回転発散+検出遅延 ~2s、屋内トラック(スパーススキャン/キッドナップのエイリアシング)、coreset M3(reuse 許容値スイープで RPE_t 回復)、ギャラリー `docs/koide_gif_gallery.md` へのアーキテクチャ結果反映。

## 関連ドキュメント

- `docs/global_localization_lio_development_plan.md` — マスタープラン(ゲート定義: coverage ≥99%、ATE ≤2.0m ほか)
- `docs/koide_gif_gallery.md` — シーケンス別の現状ギャラリー
- `experiments/koide_odometry_glim_gicp6500/` — GLIM オドメトリ単体ベンチ(全ゲート合格済み)+ coreset パッチ(M2)

## Codex 継続ログ (2026-07-18)

- 途中実装は stash せず監査して継承した。A/B/C は default-off のまま完成させ、外部
  odom seed を正常追跡中も第一候補にした。凍結 `map→odom`、拒否時 `/pcl_pose` bridge、
  G3 の bridge-first 候補、fresh fitness/N回確認、再要求の各経路を維持している。
- 低 fitness の周期構造 alias が正しい odom seed から約3 m離れて受理される事例を確認。
  opt-in の odom 補正ガードを追加し、GLIM 実験では並進 1.5 m / yaw 30 deg に設定した。
  ガード拒否を recovery supervisor が警告受理と誤分類していた不整合も修正した。
- Jazzy build と全 CTest **85/85** 合格。Humble コンテナで関連 header-only C++ 契約
  テスト合格。最新 timer 変更を含む Humble フル build は未実施。
- 140秒 A/B (score=6): baseline は output/matched 36.18%、candidate はガード調整後
  output 99.74%、matched 89.15%、RMSE 0.221 m、最大 1.144 m。
- tuned 140秒 (score=10、旧 borderline seed gate off): output **99.74%**、matched
  **96.05%**、RMSE **0.195 m**、最大 **0.414 m**。精度・coverage目標を満たすが、
  upstream alignment 自体が1.50秒空く区間があり max output gap だけ未達だった。
- この入力空白を埋めるため、bridge有効時だけ10 Hz timerで「凍結 map→odom × 最新
  GLIM odom」を `/pcl_pose` に出す変更を追加中。初版は timer の exact-time lookup が
  callback group を占有して map/scan を飢餓させたため破棄し、anchor前即return＋待ちなし
  lookupへ修正済み。修正版60秒 smoke は load<2.5待ちで未完了。
- 成果物は `/tmp/glimfrontend_runs/`。既存結果は上書きせず `.previous.<timestamp>` に
  退避している。A/B manifest は score=6 のまま、tuned結果は
  `recovery_window_glim_frontend_01a_candidate_score10.yaml` に分離した。
- 02b の退行を二分した。検証済み GLIM open-loop CSV を odom TF として入れても、従来
  localizer は 150秒で RMSE 10.35 m / 終端 30.68 m まで発散した。原因は、fitness が
  受理閾値内の小さな NDT 補正を毎回 `last_good_map_to_odom_` に凍結し、縮退区間で
  累積ドリフトさせる正帰還だった。
- default-off の map→odom anchor fitness gate を追加した。GLIM 実験は fitness≤1.5、
  seed→結果の全3D回転補正≤10 deg のときだけアンカー更新し、それ以外の「内部では
  受理」姿勢も `/pcl_pose`/TF へ直接出さず同時刻 bridge へ置換する。地面車両標準軸を
  仮定できない Livox frame 用に、外部 odom の回転を保ったまま高さだけ固定する opt-in
  (`constrain_odom_tf_prediction_height_only`) も追加した。
- 検証済みCSVの02b 150秒は最終構成の一段前で output 100%、max gap 0.501秒、並進
  RMSE **0.374 m** / 最大 1.578 m / 終端 0.145 m、姿勢 RMSE 3.05 deg / 終端 0.65 deg。
  localizer側の精度・coverage目標を満たした。stamp=0 の初期姿勢を coverage gap に含める
  evaluator bug も修正し、回帰テストを追加した。
- ライブGLIM 02b 150秒は、固定0.25秒 point delay relay を外し、直接 `/livox/points` +
  reliable sensor QoS + bag discovery delay 3秒に戻した構成が最良。開始 load 1.81 だったが
  途中で外部負荷が4.46まで上がったランでも output 100%、並進 RMSE **0.578 m** / 最大
  3.500 m / 終端 0.247 m。point delay有りは RMSE 0.869 m / 終端 0.420 m だった。
- 残るライブ固有課題は0.05〜0.5秒で40〜140 deg跳ぶ GLIM odom 瞬間姿勢と、その際の
  max output gap 0.875秒（高負荷ラン）。CSV入力では出ないため map→odom累積問題とは別。
  次は live callback/QoS順序の再現、または odom rotation continuity guard を検討する。
- 最新の関連検証: Jazzy package build成功、pose publish C++契約テスト成功、supervisor /
  ROS node / harness 76/76、bridge coverage 3/3、harness+coverage 9/9。Humble最終buildと
  4シーケンス最終フルランは未完了。
- 02b ライブ最終構成のフル298秒も完走（全return code 0、クラッシュ/NaNなし）。開始
  load 1.78 だが途中6〜7まで外部負荷が上がったストレス条件で、output **100%**、max
  output gap 0.759秒、並進 RMSE **1.089 m** / 最大 6.003 m / 終端 **0.188 m**。
  旧フル02bは RMSE 107.63 m / 終端185.52 mだったため累積発散は解消。NDT受理率は
  51.0%（286/561）でもbridgeが全拒否区間をカバーし、最大受理gap 71.9秒後も自然回復。
  G3は正しい odom bridge resetを1回出したがunconfirmed、BBSはshadow prime 1回＋弱候補
  3回を全て保留し、BBS `/initialpose` と偽recoveryは0。低負荷での再現ランは必要。
- 02b の低負荷正式フル再現 (`outdoor_hard_02b_final_guard_run01`, 開始 load 1.43) は
  output **100%**、max gap 0.600秒、並進 RMSE **0.632 m** / 最大 2.435 m / 終端
  **0.184 m**、姿勢 RMSE 10.60 deg / 終端0.45 deg。NDT受理は45.32%でも、191連続
  reject・最大受理gap 72.69秒をbridgeが覆い、全プロセスreturn code 0。G3/BBSも誤
  `/initialpose`・偽recoveryなし。したがって直前の高負荷1.089 m結果を正式値で更新する。
- raw GLIM rotation rateを120 deg/sでholdするdefault-off guardも実装・build・150秒A/Bしたが、
  TFの同一/近接時刻lookupが多い実経路では常時発火し、02bを並進0.578→0.656 m、姿勢
  18.8→23.2 degへ悪化させたためコードから完全撤回した。却下結果だけ
  `/tmp/glimfrontend_runs/outdoor_hard_02b_rotation_rate_guard_150` に保持する。
- 外部odom拘束は4系列共通の固定値にできないことをフルA/Bで確認した。02aは
  `height_only`＋anchor gateで RMSE 13.91 m / 終端38.45 m、`planar`＋同gateでは
  40.87 m / 終端69.92 mまで悪化した。02aの正常な受理325件中144件はfitness>1.5だが、
  補正は小さい（並進p90 0.18 m、yaw p90 1.02 deg）ため、一律fitness gateが正しい
  map→odom追従まで止めていた。02aは `planar`＋anchor gate無効で正式フル RMSE
  **1.000 m** / 最大3.637 m / 終端**0.195 m**、姿勢8.20 deg / 終端0.45 deg、output
  **100%** / max gap0.700秒へ復旧した（旧同系統フル0.838 mとも再現整合）。
- 01bは逆に `height_only`＋anchor gate有効が適合し、正式フル RMSE **0.999 m** / 最大
  2.236 m / 終端**0.219 m**、姿勢10.02 deg / 終端0.42 deg、output **100%** / max
  gap0.700秒。過去`planar`のRMSE 2.47〜3.17 m / 終端5.5〜9.2 mから改善した。
  ラッパーに `ODOM_TF_CONSTRAINT_MODE={planar,height_only,none}` を追加し、公開componentの
  default-offは維持したまま、01a/02a manifestだけ`planar`を明示、01b/02bは既定の
  `height_only`を使う。01a/02aではanchor fitness gateを無効、01b/02bでは有効にする。
  02a/01bともG3 bridge resetは正しい候補を1回だけ発行してunconfirmed、BBS候補は全保留、
  誤initialpose・偽recoveryは0。matched coverageは23.57% / 50.14%で95% gate未達だが、
  bridge output coverageは両方100%。
- 01aも同じ低負荷条件（開始load 2.36、UDPv4、reliable sensor QoS、delay 3秒）で正式
  フル380秒を再実行し、全return code 0、並進 RMSE **0.887 m** / 最大3.225 m / 終端
  **0.490 m**、姿勢 RMSE 16.37 deg / 終端0.34 deg。output **100%**、max gap
  0.700秒、NDT受理26.01%。G3はbridge reset 1回をunconfirmed、BBSは全候補を保留し、
  誤initialpose・偽recoveryなし。これで4系列の現行構成フルはすべて並進RMSE<2 mかつ
  終端<0.5 m（01a 0.887、01b 0.999、02a 1.000、02b 0.632 m）を満たした。
  次の未完了項目は01aフルの反復再現性（追加3回）とHumble最終build/test。
- 01a追加反復では、従来2-thread GLIMのrepeat01はRMSE **0.870 m**で再現した一方、
  repeat02は150〜360秒だけraw odomが最大33.38 m別解へ分岐し、終端0.483 mへ再収束
  したもののRMSE **15.66 m**となった。3回ともGLIM処理フレーム数2875、CPU/RSS、G3
  event列は同等で、誤resetや入力欠落ではない。GLIMソースを確認すると固定seedの
  `std::mt19937`を`randomgrid_sampling(..., num_threads=2)`へ渡し、GICPも2-threadだった。
  縮退時の並列処理順序で別解へ分岐するライブ非決定性と判断した。
- 再現性対策としてlivetf configのpreprocess/odometry `num_threads`をともに**1**へ固定。
  初回deterministic full (`outdoor_hard_01a_deterministic_repeat03`) はRMSE **0.908 m** /
  最大3.184 m / 終端0.437 m、output **100%** / max gap0.600秒、全return code 0、NaN・
  TF conflict・偽recoveryなし。GLIM odometryは平均28.1 ms、最大113.5 ms、最大一時queue
  13で、2872フレームを処理した。最終構成の3-repeat証明には同じ1-thread設定をあと2回
  実行する必要がある。
- 最新差分を対象にJazzyでpackage buildとCTest **85/85**（failure/error/skip 0）を確認。
  さらに一時workspaceへ現リポジトリと依存NDTだけをread-only mountした公式`ros:humble`
  コンテナで、rosdepから`--packages-up-to lidar_localization_ros2`のclean build、package
  testまで完走し、同じく**85/85**合格した。これにより以前の「Humbleフルbuild未実施」
  は解消済み。検証用一時成果物（約1.1 GB）もコンテナ権限で削除した。
- ただし1-thread化後2本目 (`outdoor_hard_01a_deterministic_repeat04`) は全process正常完走
  でもRMSE **40.695 m** / 終端144.251 mへ分岐し、1-threadだけでは再現性対策として
  不十分と判明した。repeat03/04のpreprocess入力stamp列と初期IMU推定値は同一で、GLIM
  odometry出力の差はLOOSE初期化窓内に始まる。誤BBS resetはなく、raw odomが約180秒
  以降に破綻してbridgeも誤軌跡を継続、BBS shadowはbridgeとの不整合で全保留した。
- 次の最終候補は、既存odometry単体01a全長試験を3/3で全gate通過したexact-coreset版。
  livetf configへ既検証値（size 32、reuse 0.1 m / 1 deg）を移し、01a manifestだけ
  `lidarloc/glim-ros2:jazzy-v1.2.2-coreset`を明示した。公開component既定値は変更しない。
  この構成で01a 3-repeatを最初から取り直す必要がある。
- coreset最終候補の01a repeat01 (`outdoor_hard_01a_coreset_repeat01`, 開始load 1.49)
  は全return code 0、並進RMSE **0.952 m** / 最大2.947 m / 終端0.495 m、姿勢RMSE
  16.70 deg / 終端0.41 deg。output **100%** / max gap0.700秒、matched 25.29%。G3は
  bridge resetを1回だけ発行してunconfirmed、BBS候補は全保留、偽recovery・誤BBS reset・
  crash/NaNなし。最終候補の再現性証明は1/3で、あと2回必要。
- coreset repeat02 (`outdoor_hard_01a_coreset_repeat02`, 開始load 2.07) も全return code 0、
  並進RMSE **0.849 m** / 最大3.085 m / 終端0.475 m、姿勢RMSE16.76 deg / 終端0.40 deg、
  output **100%** / max gap0.601秒、matched 25.21%。G3/BBS event列もrepeat01と同型で、
  誤reset・偽recoveryなし。最終候補の再現性証明は2/3、残り1回。
- coreset repeat03 (`outdoor_hard_01a_coreset_repeat03`, 開始load 2.13) も全return code 0、
  並進RMSE **0.772 m** / 最大3.230 m / 終端0.487 m、姿勢RMSE17.35 deg / 終端0.33 deg、
  output **100%** / max gap0.601秒、matched 24.85%。G3 bridge reset 1回はunconfirmed、
  BBSは全保留、誤reset・偽recoveryなし。
- よってcoreset最終候補の01a全長再現は**3/3成功**。3本集計は並進RMSE平均
  **0.857 m**（0.772--0.952）、最大誤差平均3.087 m（2.947--3.230）、終端平均
  **0.486 m**（0.475--0.495）、姿勢RMSE平均16.94 deg。output coverageは全て100%、
  max gap 0.601--0.700秒、matched coverageは24.85--25.29%。matched 95%目標は未達だが、
  bridgeを含む公開pose streamの99% coverage / 1秒gap / 並進RMSE 2 m上限は全反復合格。
- 4系列のGLIM実行設定を揃えるため01b/02a/02b manifestもcoreset imageを明示。最初の
  01b正式フル (`outdoor_hard_01b_coreset_final_run01`, 開始load 1.65) は全return code 0、
  並進RMSE **0.699 m** / 最大2.096 m / 終端0.221 m、姿勢RMSE9.51 deg / 終端0.31 deg、
  output **100%** / max gap0.700秒、matched 48.80%。旧正式RMSE0.999 mから改善し、
  G3 bridge resetはunconfirmed、BBSは全保留、誤reset・偽recoveryなし。
- 02a正式フル (`outdoor_hard_02a_coreset_final_run01`, 開始load 2.45) も全return code 0、
  並進RMSE **0.708 m** / 最大1.949 m / 終端0.195 m、姿勢RMSE8.87 deg / 終端0.44 deg、
  output **100%** / max gap0.700秒、matched 23.51%。旧正式RMSE1.000 mから改善。
  G3 bridge resetはunconfirmed、self-clear cross-check候補はmismatchで棄却、BBSは全保留、
  誤reset・偽recoveryなし。
- 02b正式フル (`outdoor_hard_02b_coreset_final_run01`, 開始load 1.84) も全return code 0、
  並進RMSE **0.650 m** / 最大2.638 m / 終端0.202 m、姿勢RMSE11.35 deg / 終端0.23 deg、
  output **100%** / max gap0.601秒、matched 46.95%。これでcoreset同一imageの4系列は
  すべて並進RMSE 0.650--0.952 m、output 100%、max gap 0.601--0.700秒、誤reset・
  偽recovery・crash/NaN/TF conflictなし。matched 95%目標だけは23.51--48.80%で未達。
- ただし02bの自然再追従後も、従来の`reinitialization_request_latched`は永続していた。
  latch policyへ連続正常sampleによる解除を追加し、途中の非正常sample・新規requestで
  連続数をresetする単体テストを追加した。最初の実走では、単なる5連続`ok`だけだと
  GLIMが別解へ分岐したrunでもfitness 4.30 / 補正1.80 mで誤解除し得ることを検出した
  （同runはRMSE 4.289 mで正式結果から除外）。解除sampleをさらにfitness **1.0以下**、
  補正並進 **0.5 m以下**へ限定した。保存済みCSVへの境界確認では、正常runは265.20秒で
  5連続条件を満たし、別解runは最後まで満たさない。最新Jazzy buildとCTestは**85/85**。
  この時点の残作業は厳格化後の02b実走と、その差分を含むHumble clean build/testだった。
- 厳格化後の02b再実走 (`outdoor_hard_02b_coreset_latch_clear_run04`, 開始load 2.32) は
  全return code 0、並進RMSE **0.655 m** / 最大2.307 m / 終端0.193 m、姿勢RMSE
  11.44 deg / 終端0.57 deg。output **100%** / max gap0.751秒、matched 63.0%。
  request中の自然再追従はfitness 0.937→0.561、補正0.014--0.064 mの適格sampleを
  5回連続で観測した264.0秒にだけlatchを解除し、`tracking/accept_measurement`へ復帰、
  終端まで出力継続。誤解除runは新gateでは解除条件を一度も満たさない。G3 bridge reset
  1回はunconfirmed、BBS reset・偽recovery・crash/NaN/TF conflictは0。
- 最新差分をread-only sourceからコピーした公式`ros:humble`一時コンテナでも、rosdep、
  `ndt_omp_ros2`からのclean `--packages-up-to` build、本体CTestまで完走し**85/85**合格
  （failure/error/skip 0）。コンテナは`--rm`で削除され、ホスト側一時build成果物なし。
  Jazzy/Humble、01a 3-repeat、4系列coreset、安全な自然再アタッチの必須検証は完了した。
