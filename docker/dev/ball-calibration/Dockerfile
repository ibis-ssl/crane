FROM python:3.12-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    zenity \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

# proto/ から Python protobuf モジュールを生成
COPY proto /tmp/ssl_proto
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

COPY *.py /app/
COPY web /app/web
COPY tests /app/tests
COPY scripts /app/scripts

EXPOSE 8090

ENV HTTP_PORT=8090

CMD ["python3", "/app/server.py"]
