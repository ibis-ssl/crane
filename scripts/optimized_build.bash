#!/bin/bash

# Crane ビルド時間最適化スクリプト
# 使用方法: ./scripts/optimized_build.bash [clean|test|benchmark]

set -e

WORKSPACE_ROOT="/home/ibis/workspace/ibis_ws"
LOG_DIR="$WORKSPACE_ROOT/src/crane/docs/logs/portal"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
BUILD_LOG="$LOG_DIR/build_${TIMESTAMP}.log"

# 色付きメッセージ
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}================================================${NC}"
    echo -e "${BLUE} Crane ビルド時間最適化スクリプト${NC}"
    echo -e "${BLUE}================================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# ワークスペースディレクトリの確認
check_workspace() {
    if [[ ! -d $WORKSPACE_ROOT ]]; then
        print_error "ワークスペースディレクトリが見つかりません: $WORKSPACE_ROOT"
        exit 1
    fi

    cd "$WORKSPACE_ROOT"

    if [[ ! -f "colcon.meta" ]]; then
        print_warning "colcon.meta が見つかりません。最適化設定なしでビルドします。"
    else
        print_success "colcon.meta 設定ファイルを確認しました"
    fi
}

# クリーンビルド
clean_build() {
    print_info "クリーンビルドを実行します..."

    if [[ -d "build" ]] || [[ -d "install" ]] || [[ -d "log" ]]; then
        print_info "既存のビルド成果物を削除中..."
        rm -rf build/ install/ log/
        print_success "ビルド成果物を削除しました"
    fi

    print_info "最適化されたクリーンビルドを開始..."
    print_info "並列度: $(nproc) workers"
    time colcon build --symlink-install --parallel-workers "$(nproc)" --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee "$BUILD_LOG"
}

# 通常ビルド
normal_build() {
    print_info "最適化されたビルドを実行します..."
    print_info "並列度: $(nproc) workers"
    time colcon build --symlink-install --parallel-workers "$(nproc)" 2>&1 | tee "$BUILD_LOG"
}

# テストビルド（特定パッケージのみ）
test_build() {
    local packages=("crane_basics" "crane_local_planner" "robocup_ssl_msgs")

    print_info "テストビルドを実行します（主要パッケージのみ）..."
    print_info "対象パッケージ: ${packages[*]}"
    print_info "並列度: $(nproc) workers"

    time colcon build --symlink-install --parallel-workers "$(nproc)" --packages-select "${packages[@]}" 2>&1 | tee "$BUILD_LOG"
}

# ベンチマークビルド
benchmark_build() {
    print_info "ベンチマークビルドを実行します..."

    # ベースライン測定（最適化なし）
    print_info "ベースライン測定中（最適化なし）..."
    if [[ -f "colcon.meta" ]]; then
        mv colcon.meta colcon.meta.bak
    fi

    rm -rf build/ install/ log/
    BASELINE_LOG="$LOG_DIR/baseline_${TIMESTAMP}.log"
    time colcon build --symlink-install 2>&1 | tee "$BASELINE_LOG"

    # 最適化ビルド測定
    print_info "最適化ビルド測定中..."
    if [[ -f "colcon.meta.bak" ]]; then
        mv colcon.meta.bak colcon.meta
    fi

    rm -rf build/ install/ log/
    time colcon build --symlink-install 2>&1 | tee "$BUILD_LOG"

    # 結果比較
    print_info "ベンチマーク結果:"
    echo "ベースライン: $BASELINE_LOG"
    echo "最適化版: $BUILD_LOG"
}

# ビルド結果の解析
analyze_build() {
    if [[ ! -f $BUILD_LOG ]]; then
        print_warning "ビルドログが見つかりません: $BUILD_LOG"
        return
    fi

    print_info "ビルド結果を解析中..."

    # パッケージ別ビルド時間の抽出
    local analysis_file="$LOG_DIR/analysis_${TIMESTAMP}.txt"

    {
        echo "=== ビルド時間解析 ==="
        echo "ログファイル: $BUILD_LOG"
        echo "生成日時: $(date)"
        echo ""
        
        echo "=== パッケージ別ビルド時間 ==="
        grep "Finished <<<" "$BUILD_LOG" | sort -k3 -hr
        
        echo ""
        echo "=== エラーと警告の統計 ==="
        echo "エラー数: $(grep -c "error:" "$BUILD_LOG" || echo 0)"
        echo "警告数: $(grep -c "warning:" "$BUILD_LOG" || echo 0)"
        
        echo ""
        echo "=== 最も時間のかかったパッケージ TOP5 ==="
        grep "Finished <<<" "$BUILD_LOG" | sort -k3 -hr | head -5
    } >"$analysis_file"

    print_success "解析結果を保存しました: $analysis_file"

    # 結果の表示
    print_info "ビルド時間 TOP5:"
    grep "Finished <<<" "$BUILD_LOG" | sort -k3 -hr | head -5 | while read -r line; do
        echo "  $line"
    done
}

# 使用方法の表示
show_usage() {
    echo "使用方法: $0 [オプション]"
    echo ""
    echo "オプション:"
    echo "  clean      クリーンビルドを実行"
    echo "  test       テストビルド（主要パッケージのみ）"
    echo "  benchmark  ベンチマークビルド（最適化前後の比較）"
    echo "  analyze    最新ログファイルの解析のみ実行"
    echo "  help       このヘルプを表示"
    echo ""
    echo "オプション未指定時は通常の最適化ビルドを実行します。"
}

# メイン処理
main() {
    print_header

    case "${1:-normal}" in
    "clean")
        check_workspace
        clean_build
        analyze_build
        ;;
    "test")
        check_workspace
        test_build
        analyze_build
        ;;
    "benchmark")
        check_workspace
        benchmark_build
        analyze_build
        ;;
    "analyze")
        # 最新のログファイルを解析
        LATEST_LOG=$(find "$LOG_DIR" -name "build_*.log" -type f -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
        if [[ -n $LATEST_LOG ]]; then
            BUILD_LOG="$LATEST_LOG"
            analyze_build
        else
            print_error "解析対象のログファイルが見つかりません"
            exit 1
        fi
        ;;
    "help")
        show_usage
        exit 0
        ;;
    "normal")
        check_workspace
        normal_build
        analyze_build
        ;;
    *)
        print_error "不明なオプション: $1"
        show_usage
        exit 1
        ;;
    esac

    print_success "ビルドスクリプトが完了しました"
    print_info "ログファイル: $BUILD_LOG"
    print_info "ポータルページ: $LOG_DIR/build_optimization_portal.html"
}

# スクリプト実行
main "$@"
