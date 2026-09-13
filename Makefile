.PHONY: help scenario-test-setup scenario-test-build scenario-test scenario-test-cm4 scenario-test-docker-up scenario-test-docker-down scenario-test-clean

# デフォルト設定
USE_LOCAL ?= 1
CRANE_TAG ?= local-scenario
PLANNER ?= rvo2

# CM4 in the loop 構成（cm4-sim を経路に挟む）の既定値。
# cm4-sim は Orion_CM4 側で実装中のため、これらは cm4-sim が存在するときのみ意味を持つ。
CM4_RX_DELAY_MS ?= 0
CM4_RX_JITTER_MS ?= 0
CM4_RX_LOSS_RATE ?= 0.0
CM4_TEST ?= VISIBILITY_OBSTACLE_AVOIDANCE

help:
	@echo "利用可能なターゲット:"
	@echo "  scenario-test-setup       - シナリオテスト環境の初期セットアップ（Python環境のみ）"
	@echo "  scenario-test-build       - シナリオテスト用Dockerイメージのビルド（リモートモード用）"
	@echo "  scenario-test             - シナリオテストの実行（全テストまたは TEST= で指定）"
	@echo "  scenario-test-cm4         - CM4 in the loop 構成でシナリオテストを実行（要 cm4-sim）"
	@echo "  scenario-test-docker-up   - Docker環境の起動（手動制御用）"
	@echo "  scenario-test-docker-down - Docker環境の停止（手動制御用）"
	@echo "  scenario-test-clean       - シナリオテスト環境のクリーンアップ"
	@echo ""
	@echo "環境変数:"
	@echo "  TEST=<テスト名>       - 実行するテスト名（例: TEST=STOP_ROBOT_SPEED）"
	@echo "  PLANNER=<プランナー>  - 経路計画アルゴリズム（rvo2 または visibility_graph、デフォルト: rvo2）"
	@echo "  USE_LOCAL=1           - ローカルのワークスペースを使用（デフォルト）"
	@echo "  USE_LOCAL=0           - Dockerイメージを使用（リモートモード）"
	@echo "  CRANE_TAG=<タグ>      - 使用するDockerイメージタグ（リモートモード時、デフォルト: local-scenario）"
	@echo ""
	@echo "使用例:"
	@echo "  make scenario-test-setup                                  # 初回のみ実行"
	@echo "  make scenario-test                                        # 全テスト実行"
	@echo "  make scenario-test TEST=STOP_ROBOT_SPEED                  # 個別テスト実行"
	@echo "  make scenario-test PLANNER=visibility_graph TEST=STOP_ROBOT_SPEED # VisibilityGraphで実行"
	@echo "  USE_LOCAL=0 make scenario-test                            # リモートモードで実行"
	@echo "  make scenario-test-cm4                                    # CM4 in the loop 構成で実行"
	@echo "  make scenario-test-cm4 CM4_RX_DELAY_MS=30 CM4_RX_LOSS_RATE=0.02 # 劣化を注入して実行"

scenario-test-setup:
	@echo "=== シナリオテスト環境のセットアップ ==="
	@bash scripts/scenario_test/setup_env.sh
	@echo ""
	@echo "✅ Python環境のセットアップが完了しました"
	@echo ""
	@if [ "$(USE_LOCAL)" = "0" ]; then \
		echo "リモートモード用にDockerイメージをビルドします..."; \
		bash scripts/scenario_test/build_docker.sh; \
	else \
		echo "ローカルモードを使用します（Dockerイメージのビルドはスキップ）"; \
		echo "※ ベースイメージ（ghcr.io/ibis-ssl/crane:base）が必要です"; \
	fi
	@echo ""
	@echo "テストを実行するには:"
	@echo "  make scenario-test"

scenario-test-build:
	@echo "=== シナリオテスト用Dockerイメージのビルド ==="
	@bash scripts/scenario_test/build_docker.sh

scenario-test:
	@if [ ! -d "scenario_test_env" ]; then \
		echo "エラー: Python環境がセットアップされていません"; \
		echo "先に 'make scenario-test-setup' を実行してください"; \
		exit 1; \
	fi
	@PLANNER=$(PLANNER) bash scripts/scenario_test/run_test.sh $(TEST)

# CM4 in the loop 構成でのシナリオテスト。
#
#   crane --12345 mode4--> cm4-sim --12346 mode3--> simulator-cli
#
# 設定の意味:
#   PLANNER=visibility_graph : mode 4（位置指令）を出す唯一の planner
#   CRANE_TARGET_PORT=12345  : crane の送信先。cm4-sim が待ち受ける
#   IBIS_PORT=12346          : simulator-cli を 12346 へ退避させる
#   FEEDBACK_SIM_MODE=false  : feedback を実機と同じ multicast で受ける。
#                              true のままだと crane と cm4-sim が 127.0.0.1:50100+id を
#                              奪い合い、片方が全パケットを取ってしまう
#
# 注意: cm4-sim は Orion_CM4 側で実装中。イメージが無い間はこのターゲットは失敗する。
scenario-test-cm4:
	@if [ ! -d "scenario_test_env" ]; then \
		echo "エラー: Python環境がセットアップされていません"; \
		echo "先に 'make scenario-test-setup' を実行してください"; \
		exit 1; \
	fi
	@echo "=== CM4 in the loop 構成でシナリオテストを実行 ==="
	@echo "  劣化注入: delay=$(CM4_RX_DELAY_MS)ms jitter=$(CM4_RX_JITTER_MS)ms loss=$(CM4_RX_LOSS_RATE)"
	@COMPOSE_PROFILES=cm4-loop \
		PLANNER=visibility_graph \
		CRANE_TARGET_PORT=12345 \
		IBIS_PORT=12346 \
		FEEDBACK_SIM_MODE=false \
		RX_DELAY_MS=$(CM4_RX_DELAY_MS) \
		RX_JITTER_MS=$(CM4_RX_JITTER_MS) \
		RX_LOSS_RATE=$(CM4_RX_LOSS_RATE) \
		bash scripts/scenario_test/run_test.sh $(CM4_TEST)

scenario-test-docker-up:
	@echo "=== Docker環境を起動中 ==="
	@./scripts/ensure-sim-network-confined.sh
	@if [ "$(USE_LOCAL)" = "1" ]; then \
		docker compose -f docker/scenario/docker-compose.local.yaml up -d; \
		echo "Docker環境が起動しました（ローカルモード）"; \
	else \
		CRANE_TAG=$(CRANE_TAG) docker compose -f docker/scenario/docker-compose.yaml up -d; \
		echo "Docker環境が起動しました（リモートモード）"; \
	fi
	@echo ""
	@echo "停止するには:"
	@echo "  make scenario-test-docker-down"

scenario-test-docker-down:
	@echo "=== Docker環境を停止中 ==="
	@if [ "$(USE_LOCAL)" = "1" ]; then \
		docker compose -f docker/scenario/docker-compose.local.yaml down; \
	else \
		docker compose -f docker/scenario/docker-compose.yaml down; \
	fi
	@echo "Docker環境が停止しました"

scenario-test-clean:
	@echo "=== シナリオテスト環境のクリーンアップ ==="
	@echo "Python仮想環境を削除中..."
	@rm -rf scenario_test_env
	@echo "Dockerイメージを削除中..."
	@docker rmi ghcr.io/ibis-ssl/crane:$(CRANE_TAG) 2>/dev/null || true
	@echo "ssl-log-recorderを削除中..."
	@rm -f ssl-log-recorder
	@echo "ssl-go-toolsを削除中..."
	@rm -rf ssl-go-tools
	@echo "ログファイルを削除中..."
	@rm -f *.log *.log.gz *.avi *.mp4
	@echo ""
	@echo "✅ クリーンアップが完了しました"
