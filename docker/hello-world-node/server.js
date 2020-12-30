// Datei hello-world-node/server.js
const http = require('http'), os = require('os');

http.createServer((req, res) => {
const dateTime = new Date(), load = os.loadavg(),
      doc = `<!DOCTYPE html>
<html>
  <head>
    <title>Hello world!</title>
    <meta charset="utf-8" />
  </head>
  <body>
    <h1>Hello world: node</h1>
    Serverzeit: ${dateTime}<br/>
    Serverauslastung (load): ${load[0]}
  </body>
</html>`
  res.setHeader('Content-Type', 'text/html');
  res.end(doc);
}).listen(8080);