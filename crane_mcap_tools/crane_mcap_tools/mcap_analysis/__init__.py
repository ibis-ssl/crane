"""MCAP annotation analysis tools for RoboCup SSL matches."""

from .extractor import AnnotationContext, MCAPAnnotationExtractor
from .gemini_client import AnalysisResult, GeminiAnalysisClient
from .mcap_tools import MCAP_TOOLS_SCHEMA, MCAPToolsHandler
from .report_generator import ReportGenerator

__all__ = [
    "MCAP_TOOLS_SCHEMA",
    "AnalysisResult",
    "AnnotationContext",
    "GeminiAnalysisClient",
    "MCAPAnnotationExtractor",
    "MCAPToolsHandler",
    "ReportGenerator",
]
