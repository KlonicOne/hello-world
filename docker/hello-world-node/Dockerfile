# Datei: hello-world-node/Dockerfile
FROM node:12
ENV TZ="Europe/Berlin"
COPY server.js /src/
USER node
CMD ["node", "/src/server.js"]