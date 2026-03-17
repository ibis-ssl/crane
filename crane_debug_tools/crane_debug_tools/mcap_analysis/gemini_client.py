"""Gemini API client for annotation analysis.

このモジュールは、Gemini REST APIを使用してアノテーションを解析します。
"""

import json
import logging
import os
import time
from collections.abc import Callable
from dataclasses import dataclass

from .extractor import AnnotationContext
from .mcap_tools import MCAPToolsHandler, MCAP_TOOLS_SCHEMA

logger = logging.getLogger(__name__)


@dataclass
class AnalysisResult:
    """Gemini APIによる解析結果."""

    root_cause: str
    tactical_analysis: str
    improvements: list[dict[str, str]]
    confidence: str
    raw_response: str = ""
    error: str | None = None

    @staticmethod
    def error_result(error: str, raw_response: str = "") -> "AnalysisResult":
        """エラー結果を生成するファクトリメソッド."""
        return AnalysisResult(
            root_cause="",
            tactical_analysis="",
            improvements=[],
            confidence="UNKNOWN",
            raw_response=raw_response,
            error=error,
        )


class GeminiAnalysisClient:
    """Gemini REST APIクライアント."""

    def __init__(
        self,
        api_key: str | None = None,
        model: str = "gemini-2.0-flash-exp",
        rate_limit_delay: float = 0.5,
    ):
        """
        初期化.

        Args:
            api_key: Gemini APIキー（Noneの場合は環境変数GEMINI_API_KEYから取得）
            model: 使用するGeminiモデル
            rate_limit_delay: API呼び出し間の遅延（秒）
        """
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError(
                "Gemini API key not provided. Set GEMINI_API_KEY environment variable "
                "or pass api_key parameter."
            )

        self.model = model
        self.rate_limit_delay = rate_limit_delay

        # google-genai SDKを遅延インポート
        try:
            from google import genai
            from google.genai import types

            self._genai = genai
            self._types = types
            self._client = genai.Client(api_key=self.api_key)
        except ImportError as e:
            raise ImportError(
                "google-genai SDK is required. Install with: pip install google-genai"
            ) from e

        logger.info(f"Initialized Gemini client with model: {model}")

    def analyze_annotation(
        self, prompt: str, system_instruction: str
    ) -> AnalysisResult:
        """
        単一のアノテーションを解析.

        Args:
            prompt: 解析プロンプト
            system_instruction: システムインストラクション

        Returns:
            解析結果
        """
        try:
            # Gemini APIを呼び出し
            response = self._client.models.generate_content(
                model=self.model,
                contents=prompt,
                config=self._types.GenerateContentConfig(
                    system_instruction=system_instruction,
                    temperature=0.2,  # 一貫性を重視
                    response_mime_type="application/json",  # JSON形式を要求
                ),
            )

            raw_response = response.text
            logger.debug(f"Gemini response: {raw_response}")

            # JSONをパース
            result_data = json.loads(raw_response)

            return AnalysisResult(
                root_cause=result_data.get("root_cause", ""),
                tactical_analysis=result_data.get("tactical_analysis", ""),
                improvements=result_data.get("improvements", []),
                confidence=result_data.get("confidence", "UNKNOWN"),
                raw_response=raw_response,
            )

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse Gemini response as JSON: {e}")
            return AnalysisResult.error_result(
                f"JSON parse error: {e}",
                raw_response if "raw_response" in locals() else "",
            )

        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            return AnalysisResult.error_result(str(e))

    def analyze_annotation_with_tools(
        self,
        annotation: AnnotationContext,
        prompt: str,
        system_instruction: str,
        max_tool_calls: int = 10,
    ) -> AnalysisResult:
        """
        ツール使用を有効にしてアノテーションを解析.

        Args:
            annotation: アノテーションコンテキスト
            prompt: 解析プロンプト
            system_instruction: システムインストラクション
            max_tool_calls: 最大ツール呼び出し回数

        Returns:
            解析結果
        """
        try:
            # ツールハンドラーを初期化
            tools_handler = MCAPToolsHandler(annotation)

            # ツール定義を作成
            tools = [
                self._types.Tool(
                    function_declarations=[
                        self._types.FunctionDeclaration(
                            name=tool["name"],
                            description=tool["description"],
                            parameters=tool["parameters"],
                        )
                        for tool in MCAP_TOOLS_SCHEMA
                    ]
                )
            ]

            # ToolConfigでANYモードを指定（ツール使用を強制）
            tool_config = self._types.ToolConfig(
                function_calling_config=self._types.FunctionCallingConfig(
                    mode="ANY",
                    allowed_function_names=[tool["name"] for tool in MCAP_TOOLS_SCHEMA],
                )
            )

            # 会話履歴
            messages = [prompt]
            tool_call_count = 0

            logger.info("Starting analysis with tool calling enabled...")

            while tool_call_count < max_tool_calls:
                # Gemini APIを呼び出し
                # Function Calling使用時はresponse_mime_typeを指定しない
                # 最初の呼び出しではANYモードでツール使用を強制
                config_params = {
                    "system_instruction": system_instruction,
                    "temperature": 0.2,
                    "tools": tools,
                }

                # 最初の呼び出しでツール使用を強制
                if tool_call_count == 0:
                    config_params["tool_config"] = tool_config

                response = self._client.models.generate_content(
                    model=self.model,
                    contents=messages,
                    config=self._types.GenerateContentConfig(**config_params),
                )

                # Function callがあるかチェック
                has_function_call = any(
                    hasattr(part, "function_call") and part.function_call
                    for part in response.candidates[0].content.parts
                )

                if has_function_call:
                    # まずGeminiのレスポンスを追加
                    messages.append(response.candidates[0].content)

                    # 全てのfunction callsを処理
                    function_response_parts = []
                    for part in response.candidates[0].content.parts:
                        if hasattr(part, "function_call") and part.function_call:
                            tool_call_count += 1
                            function_call = part.function_call

                            function_name = function_call.name
                            function_args = dict(function_call.args)

                            logger.info(
                                f"Tool call #{tool_call_count}: {function_name}({function_args})"
                            )

                            # ツールを実行
                            tool_result = tools_handler.handle(
                                function_name, function_args
                            )

                            logger.debug(f"Tool result: {tool_result}")

                            # function responseを作成
                            function_response_parts.append(
                                self._types.Part(
                                    function_response=self._types.FunctionResponse(
                                        name=function_name,
                                        response=tool_result,
                                    )
                                )
                            )

                    # 全てのfunction responsesを追加
                    function_response_content = self._types.Content(
                        role="user",
                        parts=function_response_parts,
                    )
                    messages.append(function_response_content)

                    # 次のイテレーションへ
                    continue

                # テキストレスポンスを取得
                raw_response = response.text
                logger.debug(f"Gemini final response: {raw_response}")

                # マークダウンのコードブロックを削除（```json ... ```）
                json_text = raw_response.strip()
                if json_text.startswith("```json"):
                    json_text = json_text[7:]  # ```json を削除
                if json_text.startswith("```"):
                    json_text = json_text[3:]  # ``` を削除
                if json_text.endswith("```"):
                    json_text = json_text[:-3]  # ``` を削除
                json_text = json_text.strip()

                # JSONをパース
                result_data = json.loads(json_text)

                logger.info(f"Analysis completed with {tool_call_count} tool calls")

                return AnalysisResult(
                    root_cause=result_data.get("root_cause", ""),
                    tactical_analysis=result_data.get("tactical_analysis", ""),
                    improvements=result_data.get("improvements", []),
                    confidence=result_data.get("confidence", "UNKNOWN"),
                    raw_response=raw_response,
                )

            # 最大呼び出し回数に達した
            logger.warning(f"Reached max tool calls ({max_tool_calls})")
            return AnalysisResult.error_result(
                f"Reached max tool calls ({max_tool_calls})"
            )

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse Gemini response as JSON: {e}")
            return AnalysisResult.error_result(
                f"JSON parse error: {e}",
                raw_response if "raw_response" in locals() else "",
            )

        except Exception as e:
            logger.exception(f"Gemini API error: {e}")
            return AnalysisResult.error_result(str(e))

    def _run_batch(
        self,
        items: list,
        analyze_fn: Callable[..., AnalysisResult],
        total: int,
    ) -> list[AnalysisResult]:
        """バッチ処理の共通ループ（レート制限付き）."""
        results: list[AnalysisResult] = []
        for i, item in enumerate(items):
            logger.info(f"Analyzing annotation {i + 1}/{total}...")
            result = analyze_fn(*item)
            results.append(result)
            if i < total - 1:
                time.sleep(self.rate_limit_delay)
        return results

    def analyze_batch(self, prompts: list[tuple[str, str]]) -> list[AnalysisResult]:
        """
        複数のアノテーションをバッチ解析.

        Args:
            prompts: (prompt, system_instruction)のタプルのリスト

        Returns:
            解析結果のリスト
        """
        return self._run_batch(prompts, self.analyze_annotation, len(prompts))

    def analyze_batch_with_tools(
        self,
        annotations: list[AnnotationContext],
        prompts: list[tuple[str, str]],
        max_tool_calls: int = 10,
    ) -> list[AnalysisResult]:
        """
        複数のアノテーションをツール使用有効でバッチ解析.

        Args:
            annotations: アノテーションコンテキストのリスト
            prompts: (prompt, system_instruction)のタプルのリスト
            max_tool_calls: 最大ツール呼び出し回数

        Returns:
            解析結果のリスト
        """
        items = [
            (annotation, prompt, system_instruction, max_tool_calls)
            for annotation, (prompt, system_instruction) in zip(annotations, prompts)
        ]
        return self._run_batch(items, self.analyze_annotation_with_tools, len(items))
