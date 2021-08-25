import base64
import json
import os.path
import pathlib
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DIRECTORY = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(DIRECTORY))
# workaround for autopep8 moving imports to the top.
if 'listen_to' not in sys.modules:
    from communication.listener import MessageListener, listen_to


host_name = 'localhost'
server_port = 8080

root_path = os.path.join(pathlib.Path(__file__).parent, 'debug-ui')

debug_data: dict = None
message_listener: MessageListener


class RequestHandler(BaseHTTPRequestHandler):
    routes = {
        '/': lambda self: self.send_file('/index.html'),
        '/debug-data': lambda self: self.send_debug_data()
    }

    def do_GET(self):
        if self.path in self.routes:
            self.routes[self.path](self)
        else:
            self.send_file(self.path)

    def get_absolute_path_from_relative(self, path):
        return os.path.join(root_path, path[1:])

    def send_file(self, path):
        try:
            file_to_open = self.get_absolute_path_from_relative(path)
            content_to_send = open(file_to_open).read()
            self.send_response(200)
        except:
            content_to_send = 'File not found'
            self.send_response(404)

        self.end_headers()
        self.wfile.write(bytes(content_to_send, 'utf-8'))

    def send_debug_data(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.end_headers()
        self.wfile.write(
            bytes(json.dumps(self.get_debug_data(), default=str), 'utf-8'))

    def get_debug_data(self):
        global debug_data
        if (debug_data is not None):
            debug_data["cameraFeed"] = self.get_camera_data()

        return debug_data

    def get_camera_data(self):
        try:
            frame_path = self.get_absolute_path_from_relative(
                "/current-frame.jpg")
            return base64.b64encode(
                open(frame_path, "rb").read()).decode("utf-8")
        except Exception as e:
            print(e)
            return None


def listen_to_socket():
    global message_listener
    message_listener = listen_to('perception/sensors', receive_debug_data)


def receive_debug_data(data: dict):
    global debug_data
    debug_data = data


if __name__ == '__main__':
    listen_to_socket()

    try:
        with ThreadingHTTPServer((host_name, server_port), RequestHandler) as server:
            print('Server listening on port: {}'.format(server_port))
            server.serve_forever()
    except:
        message_listener.stop()
