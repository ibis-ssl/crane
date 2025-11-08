.PHONY: help scenario-test-setup scenario-test-build scenario-test scenario-test-docker-up scenario-test-docker-down scenario-test-clean

# デフォルト設定
USE_LOCAL ?= 1
CRANE_TAG ?= local-scenario

help:
	@echo "利用可能なターゲット:"
	@echo "  scenario-test-setup       - シナリオテスト環境の初期セットアップ（Python環境のみ）"
	@echo "  scenario-test-build       - シナリオテスト用Dockerイメージのビルド（リモートモード用）"
	@echo "  scenario-test             - シナリオテストの実行（全テストまたは TEST= で指定）"
	@echo "  scenario-test-docker-up   - Docker環境の起動（手動制御用）"
	@echo "  scenario-test-docker-down - Docker環境の停止（手動制御用）"
	@echo "  scenario-test-clean       - シナリオテスト環境のクリーンアップ"
	@echo ""
	@echo "環境変数:"
	@echo "  TEST=<テスト名>  - 実行するテスト名（例: TEST=STOP_ROBOT_SPEED）"
	@echo "  USE_LOCAL=1      - ローカルのワークスペースを使用（デフォルト）"
	@echo "  USE_LOCAL=0      - Dockerイメージを使用（リモートモード）"
	@echo "  CRANE_TAG=<タグ> - 使用するDockerイメージタグ（リモートモード時、デフォルト: local-scenario）"
	@echo ""
	@echo "使用例:"
	@echo "  make scenario-test-setup                  # 初回のみ実行"
	@echo "  make scenario-test                        # 全テスト実行"
	@echo "  make scenario-test TEST=STOP_ROBOT_SPEED  # 個別テスト実行"
	@echo "  USE_LOCAL=0 make scenario-test            # リモートモードで実行"

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
	@bash scripts/scenario_test/run_test.sh $(TEST)

scenario-test-docker-up:
	@echo "=== Docker環境を起動中 ==="
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
