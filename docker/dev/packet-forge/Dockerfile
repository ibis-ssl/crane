FROM python:3.12-slim

WORKDIR /app

COPY crane_packet_forge/requirements.txt /app/
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --default-timeout=1000 --retries 10 -r requirements.txt

COPY crane_packet_forge/crane_packet_forge /app/crane_packet_forge
COPY crane_packet_forge/web /app/web

# 共有テーマと同じフォントを使う。web-debugger と同じ取得スクリプト。
COPY crane_web_debugger/web/download_fonts.py /tmp/download_fonts.py
RUN python3 /tmp/download_fonts.py --output-dir /app/fonts && rm /tmp/download_fonts.py

EXPOSE 8094

ENV HTTP_PORT=8094

CMD ["sh", "-c", "exec python3 -m crane_packet_forge.cli serve --host 0.0.0.0 --port ${HTTP_PORT} --web-root /app/web --shared-root /app/shared"]
