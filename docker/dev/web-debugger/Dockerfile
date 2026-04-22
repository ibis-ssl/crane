FROM python:3.12-slim

WORKDIR /app

COPY crane_web_debugger/web_server/requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

COPY crane_web_debugger/web_server/app.py /app/
COPY crane_web_debugger/web /app/web
RUN python3 /app/web/download_fonts.py --output-dir /app/fonts

EXPOSE 8090

ENV HTTP_PORT=8090

CMD ["sh", "-c", "exec python3 /app/app.py --host 0.0.0.0 --port ${HTTP_PORT} --web-root /app/web"]
