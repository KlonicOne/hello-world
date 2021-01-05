#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
import os
import datetime


class myServer(BaseHTTPRequestHandler):
    def do_GET(self):
        load = os.getloadavg()
        html = """<!DOCTYPE html>
<html>
    <head>
        <title>Hello World!</title>
        <meta charset="utf-8"/>
    </head>
<body>
    <h1>Hello world: Python</h1>
    Serverzeit: {now}<br/>
    Serverauslastung (load): {load}
</body>
</html>""".format(now=datetime.datetime.now().astimezone(), load=load[0])

        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(bytes(html, "utf-8"))
        return


def run():
    addr = ('', 8080)
    httpd = HTTPServer(addr, myServer)
    httpd.serve_forever()


run()
