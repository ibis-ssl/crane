FROM python:3.12-slim

WORKDIR /app

COPY server.py /app/server.py
COPY web /app/web

EXPOSE 8090

ENV HTTP_PORT=8090

CMD ["python3", "/app/server.py"]
