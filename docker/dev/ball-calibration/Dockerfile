FROM python:3.12-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    zenity \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

COPY docker/dev/ball-calibration/requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# proto/ から Python protobuf モジュールを生成
COPY docker/dev/ball-calibration/proto /tmp/ssl_proto
RUN mkdir -p /app/ssl_proto && \
    protoc -I /tmp/ssl_proto --python_out=/app/ssl_proto \
        /tmp/ssl_proto/ssl_vision_wrapper_tracked.proto \
        /tmp/ssl_proto/ssl_vision_detection_tracked.proto \
        /tmp/ssl_proto/ssl_gc_common.proto \
        /tmp/ssl_proto/ssl_gc_geometry.proto \
        /tmp/ssl_proto/ssl_vision_wrapper.proto \
        /tmp/ssl_proto/ssl_vision_detection.proto \
        /tmp/ssl_proto/ssl_vision_geometry.proto && \
    touch /app/ssl_proto/__init__.py && \
    rm -rf /tmp/ssl_proto

COPY docker/dev/ball-calibration/*.py /app/
COPY docker/dev/ball-calibration/web /app/web
COPY docker/dev/ball-calibration/tests /app/tests
COPY docker/dev/ball-calibration/scripts /app/scripts

# フォントをビルド時にイメージへ焼き込む (web-debugger と同じスクリプトを共用)
COPY crane_web_debugger/web/download_fonts.py /tmp/download_fonts.py
RUN python3 /tmp/download_fonts.py --output-dir /app/fonts && rm /tmp/download_fonts.py

EXPOSE 8090

ENV HTTP_PORT=8090

CMD ["python3", "/app/server.py"]
