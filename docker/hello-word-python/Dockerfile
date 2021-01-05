# Datei : hello-world-python/Dockerfile (docbuc/hello-world-python)
FROM python:3
ENV TZ="Europe/Berlin"
COPY server.py /src/
USER www-data
CMD ["python", "/src/server.py"]
