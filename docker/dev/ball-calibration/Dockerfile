FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt /app/
RUN pip install --no-cache-dir -r requirements.txt

COPY *.py /app/
COPY web /app/web
COPY tests /app/tests
COPY scripts /app/scripts

EXPOSE 8090

ENV HTTP_PORT=8090

CMD ["python3", "/app/server.py"]
